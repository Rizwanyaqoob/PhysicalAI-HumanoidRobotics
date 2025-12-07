import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './RoboticsConceptCard.module.css';
interface Concept {
  title: string;
  description: string;
  icon: string;     
  link: string;
}

const ConceptList: Concept[] = [
  {
    title: 'Perception Systems',
    description: 'Computer vision, sensor fusion, and state estimation for humanoid robots.',
    icon: '👁️',
    link: '/docs/docs/perception/',
  },
  {
    title: 'Motion Planning',
    description: 'Path planning, trajectory optimization, and locomotion for humanoid robots.',
    icon: '🧭',
    link: '/docs/docs/motion-planning/',
  },
  {
    title: 'Reinforcement Learning',
    description: 'AI techniques for learning complex robotic behaviors through interaction.',
    icon: '🧠',
    link: '/docs/reinforcement-learning/',
  },
  {
    title: 'Testing & Debugging',
    description: 'Comprehensive methodologies for robust and reliable robotic systems.',
    icon: '🔧',
    link: '/docs/docs/testing-debugging/',
  },
  {
    title: 'Physical AI',
    description: 'AI systems that sense, act, and adapt through real-world physical interaction.',
    icon: '🌐',
    link: '/docs/docs/foundations/physical-ai',
  },
  {
    title: 'Humanoid Control',
    description: 'Control models for balance, locomotion, whole-body control, and manipulation.',
    icon: '🦾',
    link: '/docs/docs/ros2/humanoid-control',
  },
];

interface ConceptProps extends Concept {}

function Concept({ icon, title, description, link }: ConceptProps) {
  return (
    <div className="col col--4">
      <Link
        className={clsx('card', styles.conceptCard)}
        to={link}
        aria-label={`${title}: ${description}`}>
        <div className="card__header text--center">
          <span
            className={styles.conceptIcon}
            aria-label={title}
            role="img">
            {icon}
          </span>
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default function RoboticsConceptCards() {
  return (
    <section className={styles.concepts}>
      <div className="container">
        <div className="row">
          {ConceptList.map((concept, index) => (
            <Concept key={index} {...concept} />
          ))}
        </div>
      </div>
    </section>
  );
}
