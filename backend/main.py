"""
Website Content Extraction and RAG Ingestion Pipeline

This script implements a complete pipeline to:
1. Crawl a Docusaurus website
2. Extract text content from pages
3. Chunk the content
4. Generate embeddings using Cohere
5. Store vectors in Qdrant Cloud
"""

import os
import requests
from urllib.parse import urljoin, urlparse
from bs4 import BeautifulSoup
import trafilatura
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import logging
from typing import List, Dict, Tuple
import re

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RAGIngestionPipeline:
    def __init__(self):
        # Initialize API clients
        self.cohere_client = cohere.Client(os.getenv('COHERE_API_KEY'))
        self.qdrant_client = QdrantClient(
            url=os.getenv('QDRANT_URL'),
            api_key=os.getenv('QDRANT_API_KEY'),
        )
        self.target_site_url = os.getenv('TARGET_SITE_URL', 'https://rizwanyaqoob.github.io/PhysicalAI-HumanoidRobotics/')

        # Set up session for requests
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })

        # Track visited URLs to avoid duplicates
        self.visited_urls = set()

        # Create Qdrant collection for RAG embeddings
        self.create_collection()

    def get_all_urls(self, base_url: str) -> List[str]:
        """
        Crawl the deployed Docusaurus site and collect unique URLs.
        Uses sitemap if available, otherwise performs internal link discovery.
        """
        logger.info(f"Starting to crawl: {base_url}")
        urls = set()

        # First, try to get URLs from sitemap
        sitemap_url = urljoin(base_url, 'sitemap.xml')
        sitemap_urls = self._get_urls_from_sitemap(sitemap_url)
        urls.update(sitemap_urls)

        if not sitemap_urls:
            logger.info("No sitemap found, performing internal link discovery")
            # If no sitemap, perform breadth-first crawl of internal links
            urls.update(self._crawl_internal_links(base_url))

        logger.info(f"Found {len(urls)} unique URLs to process")
        return list(urls)

    def _get_urls_from_sitemap(self, sitemap_url: str) -> List[str]:
        """Extract URLs from sitemap.xml if available."""
        try:
            response = self.session.get(sitemap_url, timeout=10)
            if response.status_code == 200:
                soup = BeautifulSoup(response.content, 'xml')
                urls = []
                for loc in soup.find_all('loc'):
                    url = loc.text.strip()
                    if url.startswith(self.target_site_url):
                        urls.append(url)
                logger.info(f"Retrieved {len(urls)} URLs from sitemap")
                return urls
        except Exception as e:
            logger.warning(f"Could not fetch sitemap: {e}")

        return []

    def _crawl_internal_links(self, base_url: str, max_pages: int = 200) -> List[str]:
        """Perform internal link discovery to find all site URLs."""
        urls = set()
        to_visit = [base_url]

        while to_visit and len(urls) < max_pages:
            current_url = to_visit.pop(0)

            if current_url in self.visited_urls:
                continue

            self.visited_urls.add(current_url)

            try:
                response = self.session.get(current_url, timeout=10)
                if response.status_code == 200:
                    urls.add(current_url)
                    logger.info(f"Crawled: {current_url}")

                    # Find internal links
                    soup = BeautifulSoup(response.content, 'html.parser')
                    for link in soup.find_all('a', href=True):
                        href = link['href']
                        full_url = urljoin(current_url, href)

                        # Only add URLs that are part of the target site
                        if (full_url.startswith(self.target_site_url) and
                            full_url not in self.visited_urls and
                            full_url not in to_visit):
                            to_visit.append(full_url)

            except Exception as e:
                logger.error(f"Error crawling {current_url}: {e}")

        return list(urls)

    def extract_text_from_url(self, url: str) -> Tuple[str, str]:
        """
        Extract and clean page text using trafilatura with fallback HTML parsing.
        Returns (title, content) tuple.
        """
        try:
            response = self.session.get(url, timeout=10)
            if response.status_code != 200:
                logger.error(f"Failed to fetch {url}, status code: {response.status_code}")
                return "", ""

            # Use trafilatura for primary extraction
            content = trafilatura.extract(response.text, include_comments=False,
                                         include_tables=True, include_formatting=True)

            # If trafilatura fails, use BeautifulSoup as fallback
            if not content:
                soup = BeautifulSoup(response.text, 'html.parser')

                # Remove script and style elements
                for script in soup(["script", "style"]):
                    script.decompose()

                # Extract title
                title_tag = soup.find('title')
                title = title_tag.get_text().strip() if title_tag else ""

                # Extract main content - try common content containers
                content_selectors = [
                    'main', '.main-content', '.content', '#content',
                    '.post', '.article', '.markdown', '.doc-content'
                ]

                content = ""
                for selector in content_selectors:
                    element = soup.select_one(selector)
                    if element:
                        content = element.get_text(separator=' ', strip=True)
                        break

                # If no specific content container found, extract from body
                if not content:
                    body = soup.find('body')
                    if body:
                        content = body.get_text(separator=' ', strip=True)

            # Extract title if not already done
            if not content:
                soup = BeautifulSoup(response.text, 'html.parser')
                title_tag = soup.find('title')
                title = title_tag.get_text().strip() if title_tag else ""
            else:
                # Extract title from the content if not done via trafilatura
                soup = BeautifulSoup(response.text, 'html.parser')
                title_tag = soup.find('title')
                title = title_tag.get_text().strip() if title_tag else ""

            # Clean up content
            content = re.sub(r'\s+', ' ', content).strip()

            return title, content

        except Exception as e:
            logger.error(f"Error extracting text from {url}: {e}")
            return "", ""

    def chunk_text(self, text: str, max_tokens: int = 800) -> List[Dict]:
        """
        Chunk text into 500–1000-token segments with metadata.
        Returns list of chunks with metadata (url, title, chunk index).
        """
        if not text:
            return []

        # Rough estimation: 1 token ≈ 4 characters for English text
        max_chars = max_tokens * 4

        chunks = []
        paragraphs = text.split('\n\n')

        current_chunk = ""
        current_chunk_size = 0

        for i, paragraph in enumerate(paragraphs):
            paragraph_size = len(paragraph)

            # If adding this paragraph would exceed the limit
            if current_chunk_size + paragraph_size > max_chars and current_chunk:
                # Save the current chunk
                chunks.append({
                    'content': current_chunk.strip(),
                    'size': current_chunk_size
                })
                current_chunk = paragraph
                current_chunk_size = paragraph_size
            else:
                # Add paragraph to current chunk
                if current_chunk:
                    current_chunk += "\n\n" + paragraph
                    current_chunk_size += paragraph_size + 2  # +2 for the \n\n
                else:
                    current_chunk = paragraph
                    current_chunk_size = paragraph_size

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                'content': current_chunk.strip(),
                'size': current_chunk_size
            })

        # Further split large chunks if needed
        final_chunks = []
        for chunk in chunks:
            if len(chunk['content']) > max_chars:
                # Split large chunk into smaller ones
                sub_chunks = self._split_large_chunk(chunk['content'], max_chars)
                final_chunks.extend(sub_chunks)
            else:
                final_chunks.append(chunk)

        return final_chunks

    def _split_large_chunk(self, text: str, max_chars: int) -> List[Dict]:
        """Split a large chunk into smaller ones based on sentences."""
        sentences = re.split(r'[.!?]+', text)
        chunks = []
        current_chunk = ""
        current_size = 0

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            sentence_size = len(sentence)

            if current_size + sentence_size > max_chars and current_chunk:
                chunks.append({
                    'content': current_chunk.strip(),
                    'size': current_size
                })
                current_chunk = sentence
                current_size = sentence_size
            else:
                if current_chunk:
                    current_chunk += ". " + sentence
                    current_size += sentence_size + 2  # +2 for ". "
                else:
                    current_chunk = sentence
                    current_size = sentence_size

        if current_chunk.strip():
            chunks.append({
                'content': current_chunk.strip(),
                'size': current_size
            })

        return chunks

    def embed(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings with Cohere in batches.
        """
        if not texts:
            return []

        try:
            # Cohere has a limit on the number of texts per request
            batch_size = 96  # Safe limit below Cohere's max
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                response = self.cohere_client.embed(
                    texts=batch,
                    model="embed-english-v3.0",  # Latest model as of 2024
                    input_type="search_document"
                )
                all_embeddings.extend(response.embeddings)

                # Add small delay to avoid rate limiting
                time.sleep(0.1)

            logger.info(f"Generated embeddings for {len(all_embeddings)} text chunks")
            return all_embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            return []

    def create_collection(self, collection_name: str = "RAG_embeddings"):
        """
        Create Qdrant collection for RAG embeddings.
        """
        try:
            # Check if collection already exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == collection_name for col in collections.collections)

            if not collection_exists:
                # Create new collection
                self.qdrant_client.create_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
                )
                logger.info(f"Created Qdrant collection: {collection_name}")
            else:
                logger.info(f"Qdrant collection {collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")

    def save_chunk_to_qdrant(self, chunk_data: Dict, embedding: List[float], collection_name: str = "RAG_embeddings"):
        """
        Save a text chunk with its embedding to Qdrant.
        """
        try:
            import uuid

            # Prepare payload with metadata
            payload = {
                "url": chunk_data.get("url", ""),
                "title": chunk_data.get("title", ""),
                "chunk_index": chunk_data.get("chunk_index", 0),
                "content_preview": chunk_data.get("content", "")[:200],  # First 200 chars as preview
                "original_content_length": len(chunk_data.get("content", ""))
            }

            # Generate a proper UUID for the point ID
            point_id = str(uuid.uuid4())

            # If we want to preserve the original URL/chunk info in the ID, we can create a more descriptive ID
            # But for Qdrant compatibility, we'll use a UUID
            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload
            )

            # Upsert the point to Qdrant
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=[point]
            )

            logger.debug(f"Saved chunk to Qdrant: {chunk_data.get('url', '')} - chunk {chunk_data.get('chunk_index', 0)} with ID {point_id}")

        except Exception as e:
            logger.error(f"Error saving chunk to Qdrant: {e}")

    def run_pipeline(self):
        """
        Execute the complete ingestion pipeline.
        """
        logger.info("Starting RAG ingestion pipeline...")

        # Step 1: Get all URLs from the target site
        urls = self.get_all_urls(self.target_site_url)

        total_pages = len(urls)
        processed_pages = 0
        total_chunks = 0

        logger.info(f"Starting to process {total_pages} pages")

        for i, url in enumerate(urls):
            logger.info(f"Processing page {i+1}/{total_pages}: {url}")

            # Step 2: Extract text content from URL
            title, content = self.extract_text_from_url(url)

            if not content:
                logger.warning(f"No content extracted from {url}")
                continue

            # Step 3: Chunk the content
            chunks = self.chunk_text(content)

            # Process each chunk
            for j, chunk in enumerate(chunks):
                chunk_data = {
                    "id": f"{url_hash(url)}_{j}",
                    "url": url,
                    "title": title,
                    "content": chunk['content'],
                    "chunk_index": j
                }

                # Step 4: Generate embedding for the chunk
                embeddings = self.embed([chunk['content']])

                if embeddings and len(embeddings) > 0:
                    # Step 5: Save to Qdrant
                    self.save_chunk_to_qdrant(chunk_data, embeddings[0])
                    total_chunks += 1
                else:
                    logger.error(f"Failed to generate embedding for chunk {j} of {url}")

            processed_pages += 1

            # Log progress periodically
            if processed_pages % 10 == 0:
                logger.info(f"Progress: {processed_pages}/{total_pages} pages processed, {total_chunks} chunks saved")

        logger.info(f"Pipeline completed! Processed {processed_pages} pages and saved {total_chunks} chunks to Qdrant.")


def url_hash(url: str) -> str:
    """Generate a simple hash for URL to use in IDs."""
    import hashlib
    return hashlib.md5(url.encode()).hexdigest()[:12]


def main():
    """
    Main function to execute the RAG ingestion pipeline.
    """
    logger.info("Initializing RAG Ingestion Pipeline")

    # Validate environment variables
    required_vars = ['COHERE_API_KEY', 'QDRANT_API_KEY', 'QDRANT_URL']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        return

    # Create and run the pipeline
    pipeline = RAGIngestionPipeline()
    pipeline.run_pipeline()


if __name__ == "__main__":
    main()