"""
Agent service for the RAG Agent Backend using OpenAI Agents SDK with Gemini configuration
"""
import time
import logging
from typing import List, Dict, Any, Optional
from openai import OpenAI, AsyncOpenAI
from ..config.settings import settings
from ..config.logger import logger
from ..models.response_models import Chunk


class AgentService:
    """
    Service class for generating answers using OpenAI Agents SDK with Gemini configuration
    """
    def __init__(self):
        # Initialize OpenAI client with Gemini-compatible endpoint
        # Using GEMINI_API_KEY with OpenAI-compatible endpoint for Gemini
        if settings.GEMINI_API_KEY:
            self.openai_client = OpenAI(
                api_key=settings.GEMINI_API_KEY,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
            self.async_openai_client = AsyncOpenAI(
                api_key=settings.GEMINI_API_KEY,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
        else:
            self.openai_client = None
            self.async_openai_client = None

        self.logger = logger

    def generate_answer_with_retrieval_tool(self, query: str, max_tokens: int = 1000, temperature: float = 0.7) -> str:
        """
        Generate an answer using OpenAI client configured for Gemini with retrieval tool
        The agent will first retrieve relevant content and then answer based on it
        """
        if not self.openai_client:
            raise ValueError("GEMINI_API_KEY not configured")

        start_time = time.time()

        try:
            # Create the initial prompt for the agent
            system_prompt = """You are a helpful assistant that answers questions based on provided book content.
            You have access to a retrieval tool that can find relevant content from the knowledge base.
            First, use the retrieval tool to find relevant information, then answer the user's question based only on the retrieved content.
            If the retrieved content doesn't contain enough information to answer the question, respond with 'I don't know'.
            Always cite the source document when providing information from the retrieved content."""

            # First call: let the agent decide if it needs to use the retrieval tool
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": query}
            ]

            # Call OpenAI API with the tool available
            response = self.openai_client.chat.completions.create(
                model="gemini-2.0-flash",
                messages=messages,
                tools=[self.get_retrieval_tool_schema()],
                tool_choice="auto",  # Let the model decide when to use the tool
                max_tokens=max_tokens,
                temperature=temperature
            )

            # Process the response
            response_message = response.choices[0].message

            # Check if the model wants to call the tool
            tool_calls = response_message.tool_calls
            if tool_calls:
                # Add the tool call to the messages
                messages.append(response_message)

                # Execute the tool calls
                import json
                from .retrieval_service import RetrievalService
                retrieval_service = RetrievalService()

                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    if function_name == "retrieve_content":
                        function_args = json.loads(tool_call.function.arguments)
                        query = function_args.get("query", query)
                        max_chunks = function_args.get("max_chunks", 5)

                        # Call the retrieval tool
                        tool_response = retrieval_service.retrieve_content_tool(query, max_chunks)

                        # Add the tool response to the messages
                        messages.append({
                            "tool_call_id": tool_call.id,
                            "role": "tool",
                            "name": function_name,
                            "content": tool_response
                        })

                # Get the final response based on the tool results
                final_response = self.openai_client.chat.completions.create(
                    model="gemini-2.0-flash",
                    messages=messages,
                    max_tokens=max_tokens,
                    temperature=temperature
                )

                answer = final_response.choices[0].message.content
            else:
                # If no tool was called, use the initial response
                answer = response_message.content

            generation_time = time.time() - start_time

            self.logger.info(f"Generated answer in {generation_time:.2f} seconds using Gemini via OpenAI-compatible endpoint with retrieval tool")

            return answer

        except Exception as e:
            self.logger.error(f"Error generating answer with Gemini via OpenAI-compatible endpoint: {e}")
            raise

    def generate_answer(self, query: str, chunks: List[Chunk] = None,
                       max_tokens: int = 1000, temperature: float = 0.7) -> str:
        """
        Generate an answer using OpenAI client configured for Gemini with retrieval tool
        If chunks are provided, use them directly; otherwise, use the retrieval tool
        """
        if chunks is None or len(chunks) == 0:
            # Use the retrieval tool approach
            return self.generate_answer_with_retrieval_tool(query, max_tokens, temperature)
        else:
            # Use the original approach with provided chunks
            return self.generate_answer_with_context(query, chunks, max_tokens, temperature)

    def generate_answer_with_context(self, query: str, chunks: List[Chunk],
                                   max_tokens: int = 1000, temperature: float = 0.7) -> str:
        """
        Generate an answer using OpenAI client configured for Gemini based on the query and provided context chunks
        """
        if not self.openai_client:
            raise ValueError("GEMINI_API_KEY not configured")

        start_time = time.time()

        try:
            # Format the context from retrieved chunks
            context = self._format_context(chunks)

            # Create the prompt for Gemini via OpenAI-compatible endpoint
            system_prompt = """You are a helpful assistant that answers questions based on provided book content.
            Use the provided context to answer the user's question accurately.
            If the context doesn't contain enough information to answer the question, say so clearly.
            Always cite the source document when providing information from the context.
            IMPORTANT: Answer ONLY based on the provided context. If the context doesn't contain the information needed, respond with 'I don't know'."""

            user_prompt = f"""Context:\n{context}\n\nQuestion: {query}"""

            # Call OpenAI API to generate the answer using gemini-2.0-flash model
            response = self.openai_client.chat.completions.create(
                model="gemini-2.0-flash",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=max_tokens,
                temperature=temperature
            )

            answer = response.choices[0].message.content
            generation_time = time.time() - start_time

            self.logger.info(f"Generated answer in {generation_time:.2f} seconds using Gemini via OpenAI-compatible endpoint")

            return answer

        except Exception as e:
            self.logger.error(f"Error generating answer with Gemini via OpenAI-compatible endpoint: {e}")
            raise


    def _format_context(self, chunks: List[Chunk]) -> str:
        """
        Format the context from retrieved chunks
        """
        formatted_chunks = []
        for chunk in chunks:
            formatted_chunk = f"Source: {chunk.source_document}"
            if chunk.page_number is not None:
                formatted_chunk += f", Page: {chunk.page_number}"
            if chunk.section_title:
                formatted_chunk += f", Section: {chunk.section_title}"
            formatted_chunk += f"\nContent: {chunk.content}\n"
            formatted_chunks.append(formatted_chunk)

        return "\n\n".join(formatted_chunks)

    def validate_provider(self, provider: str) -> bool:
        """
        Validate if the provider is supported (only gemini via OpenAI-compatible endpoint)
        """
        return provider.lower() == "gemini"

    def test_gemini_connection(self) -> bool:
        """
        Test Gemini API connection via OpenAI-compatible endpoint
        """
        if not self.openai_client:
            self.logger.warning("GEMINI_API_KEY not configured")
            return False

        try:
            response = self.openai_client.chat.completions.create(
                model="gemini-2.0-flash",
                messages=[{"role": "user", "content": "test"}],
                max_tokens=5
            )
            return True
        except Exception as e:
            self.logger.error(f"Gemini connection test via OpenAI-compatible endpoint failed: {e}")
            return False

    def get_retrieval_tool_schema(self):
        """
        Return the schema for the retrieval tool that can be used with OpenAI function calling
        """
        return {
            "type": "function",
            "function": {
                "name": "retrieve_content",
                "description": "Retrieve relevant content from the knowledge base based on the user's query",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query to find relevant content"
                        },
                        "max_chunks": {
                            "type": "integer",
                            "description": "Maximum number of content chunks to retrieve (default: 5)",
                            "default": 5
                        }
                    },
                    "required": ["query"]
                }
            }
        }