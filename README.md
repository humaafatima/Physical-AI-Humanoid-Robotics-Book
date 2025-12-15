# Physical AI & Humanoid Robotics: From Simulated Digital Twins to Conversational Embodied Agents

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Deploy Book](https://github.com/humaafatima/physical-ai-humanoid-robotics-book/actions/workflows/deploy-book.yml/badge.svg)](https://github.com/humaafatima/physical-ai-humanoid-robotics-book/actions/workflows/deploy-book.yml)

> **An open-source book and 13-week capstone course** teaching students to design, simulate, and deploy conversational humanoid robots using ROS 2, NVIDIA Isaac Sim, and Vision-Language-Action models.

## 📖 About This Book

This comprehensive guide (18,000-25,000 words, 14 chapters) bridges digital intelligence with physical embodiment, teaching you to build conversational humanoid robots from simulation through hardware deployment.

**[🌐 Read the Book Online](https://humaafatima.github.io/physical-ai-humanoid-robotics-book/)** | **[📥 Download PDF](https://github.com/humaafatima/physical-ai-humanoid-robotics-book/releases/latest)**

### What You'll Learn

- **ROS 2** for real-time robotic control and communication
- **Digital twin simulation** in Gazebo, Unity, and NVIDIA Isaac Sim
- **GPU-accelerated perception** with Isaac ROS (object detection, VSLAM)
- **Autonomous navigation** with Nav2 for bipedal robots
- **Vision-Language-Action models** for embodied AI
- **Voice control integration** using Whisper + LLMs + ROS 2
- **Sim-to-real transfer** techniques for physical hardware deployment

### Target Audience

- **Students**: Advanced CS/AI/Robotics students completing capstone projects
- **Researchers**: Entering embodied AI and humanoid robotics
- **Engineers**: Building next-generation physical AI systems
- **Instructors**: Teaching robotics and embodied AI courses

## 📚 Table of Contents

1. **Introduction to Physical AI and Embodied Intelligence**
2. **The Humanoid Robotics Landscape (2025)**
3. **ROS 2 – The Robotic Nervous System**
4. **URDF & Robot Modeling for Humanoids**
5. **Digital Twins – Gazebo, Unity, and NVIDIA Isaac Sim**
6. **Sensor Simulation and Real-World Integration**
7. **NVIDIA Isaac ROS – Accelerated Perception and VSLAM**
8. **Navigation and Bipedal Locomotion with Nav2**
9. **Vision-Language-Action Models (VLA) for Robotics**
10. **From Voice to Action – Whisper + LLM + ROS 2 Planning**
11. **Manipulation and Dexterous Grasping**
12. **Sim-to-Real Transfer Techniques**
13. **Capstone Project – Autonomous Conversational Humanoid**
14. **Hardware Guide & Lab Architectures**

## 🚀 Quick Start

### For Readers

Visit the **[online book](https://your-username.github.io/physical-ai-humanoid-robotics-book/)** and start with Chapter 1.

### For Developers

```bash
# Clone the repository
git clone https://github.com/your-org/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book

# Install dependencies
cd book
npm install

# Start local development server
npm start
# Opens http://localhost:3000

# Build for production
npm run build
```

### For Students (Code Examples)

```bash
# Navigate to a code example
cd code-examples/chapter-03-ros2/hello_ros2/

# Run with Docker
docker-compose up

# Follow README.md in each example directory
```

## 🛠️ Tech Stack

### Book Platform
- **Docusaurus 3.x**: Static site generator for technical documentation
- **React**: Component-based UI framework
- **Algolia DocSearch**: Search functionality
- **GitHub Pages**: Free hosting and deployment

### Code Examples
- **ROS 2 Humble Hawksbill (LTS)**: Robotics middleware (supported through May 2027)
- **NVIDIA Isaac Sim 2024.1+**: GPU-accelerated robotics simulation
- **NVIDIA Isaac ROS 2.x**: GPU-accelerated computer vision
- **Python 3.10+**: Primary programming language (rclpy)
- **Docker**: Containerized reproducible environments
- **Nav2**: ROS 2 navigation framework
- **MoveIt 2**: Motion planning for manipulation
- **OpenAI Whisper**: Speech-to-text
- **GPT-4 / Claude / Llama 3**: LLM integration for voice commands

## 📁 Repository Structure

```
/
├── book/                   # Docusaurus book source
│   ├── docs/              # 14 chapter markdown files
│   ├── static/            # Images, diagrams, assets
│   ├── docusaurus.config.js
│   └── package.json
├── code-examples/         # Executable ROS 2 + Isaac Sim code
│   ├── chapter-03-ros2/
│   ├── chapter-04-urdf/
│   ├── ... (one per chapter with code)
│   └── chapter-13-capstone/
├── hardware/              # Hardware BOMs and assembly guides
│   ├── economy-kit.md     # ~$700 kit
│   ├── midrange-kit.md    # ~$2k-3k kit
│   └── full-lab.md        # ~$10k+ lab
├── course-materials/      # 13-week course curriculum
│   ├── syllabus.md
│   ├── week-01/ ... week-13/
│   └── assessments/
├── references/            # Citation management
│   ├── bibliography.bib   # Zotero export (APA 7th)
│   └── citation-log.md
├── docker/                # Shared Docker base images
├── scripts/               # Utility scripts
├── .github/workflows/     # CI/CD automation
└── specs/                 # Project planning docs
```

## 🎓 For Instructors

This book is designed for a **13-week capstone course**. Instructor resources include:

- **Syllabus**: Week-by-week schedule mapped to chapters
- **Lecture slides**: Template slide decks for each week
- **Exercises**: 100+ coding and conceptual exercises
- **Grading rubrics**: Midterm checkpoint and final capstone rubrics
- **Lab setup guide**: Instructions for institutional compute clusters

See `course-materials/` directory for complete resources.

## 🤝 Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Ways to Contribute
- **Corrections**: Typos, broken links, technical errors
- **Improvements**: Clearer explanations, additional examples, better diagrams
- **Platform adaptations**: Ports to ROS 2 Jazzy, alternative simulation platforms
- **Translations**: Community-driven translations to other languages

### Code of Conduct
This project follows the [Contributor Covenant Code of Conduct](https://www.contributor-covenant.org/version/2/0/code_of_conduct/).

## 📝 Citation

If you use this book in your research or teaching, please cite:

```bibtex
@book{physicalai2025,
  title={Physical AI \& Humanoid Robotics: From Simulated Digital Twins to Conversational Embodied Agents},
  author={Physical AI Robotics Team},
  year={2025},
  publisher={GitHub},
  url={https://github.com/your-org/physical-ai-humanoid-robotics-book}
}
```

## 📄 License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) for details.

## 🔗 Links

- **[Book Website](https://your-username.github.io/physical-ai-humanoid-robotics-book/)**
- **[GitHub Discussions](https://github.com/your-org/physical-ai-humanoid-robotics-book/discussions)**: Q&A and community
- **[Issue Tracker](https://github.com/your-org/physical-ai-humanoid-robotics-book/issues)**: Bug reports and feature requests
- **[ROS 2 Documentation](https://docs.ros.org/en/humble/)**
- **[NVIDIA Isaac Sim Docs](https://docs.omniverse.nvidia.com/isaacsim/)**

## 🙏 Acknowledgments

This book builds on the work of the ROS, NVIDIA Isaac, and embodied AI communities. Special thanks to all contributors and reviewers.

---

**Status**: 🚧 **In Development** - Target completion: Week 12

**Get Started**: [Chapter 1: Introduction to Physical AI →](https://your-username.github.io/physical-ai-humanoid-robotics-book/docs/01-intro)
<!-- Trigger deployment -->
