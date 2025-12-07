import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './RoboticsConceptCard.module.css';

interface Concept {
  title: string;
  description: string;
  icon: string;     // Replace later with proper SVGs
  link: string;
}

const ConceptList: Concept[] = [
  {
    title: 'Physical AI',
    description: 'AI systems that sense, act, and adapt through real-world physical interaction.',
    icon: '🧠',
    link: '/docs/foundations/physical-ai',
  },
  {
    title: 'Embodied Intelligence',
    description: 'Intelligence shaped by geometry, environment, perception, and action.',
    icon: '🤖',
    link: '/docs/foundations/embodied-intelligence',
  },
  {
    title: 'Humanoid Control',
    description: 'Control models for balance, locomotion, whole-body control, and manipulation.',
    icon: '🦾',
    link: '/docs/ros2/humanoid-control',
  },
];

interface ConceptProps extends Concept {}

function Concept({ icon, title, description, link }: ConceptProps) {
  return (
    <div className="col col--4">
      <Link className={clsx('card', styles.conceptCard)} to={link}>
        <div className="card__header text--center">
          <span className={styles.conceptIcon} aria-hidden="true">{icon}</span>
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
