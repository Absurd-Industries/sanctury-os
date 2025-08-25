# CLAUDE.md - Project Understanding

## Project Overview
**Sanctuary OS** - A real-time wildlife conservation monitoring platform built by Absurd Industries in collaboration with The Liana Trust. This is an environmental monitoring system designed for wildlife conservation facilities.

## Project Structure
This is a frontend-focused web application with the following key components:

### Files & Architecture
- `index.html` - Main monitoring dashboard interface
- `parts.html` - Component configurator/parts picker interface
- `readme.md` - Comprehensive project documentation
- `images/` - Species photos organized by codes (CrMlKa, DaRuKa, EcCaKa, HyHyKa, OpKaKa)
- `videos/` - Corresponding video content for each species

### Technology Stack
- **Frontend**: Vue.js 3, Tailwind CSS, FontAwesome
- **Styling**: Custom conservation-themed color palette (radium, laboratory, uranium, polonium, beige variants)
- **Design**: Glass-morphism UI with responsive mobile-first approach
- **Typography**: Rubik font family

### Purpose & Domain
This is a **conservation technology project** focused on:
- Real-time environmental monitoring (temperature, humidity, UV)
- Wildlife habitat management
- Scientific data collection for research
- Mobile-friendly interface for field use
- Species-specific care tracking

### Key Features (from documentation)
- Environmental sensor integration (temperature ±0.1°C, humidity ±2%)
- Live camera feeds for visual monitoring
- Species-specific optimal range indicators
- Care schedule tracking and research documentation
- Export capabilities for academic collaboration

### Development Context
- Open source project (GPL v3 license)
- Built for conservation facilities and educational institutions
- Hardware integration planned (ESP32, DHT22 sensors, etc.)
- Demo available at sanctuary.absurd.industries

## Claude's Role
When working on this project, I should:
- Prioritize conservation and animal welfare considerations
- Maintain the scientific/research focus
- Respect the mobile-first, accessible design principles
- Follow the established color scheme and Vue.js patterns
- Consider real-world conservation facility workflows
- Support both educational and research use cases

## Testing & Quality
- No specific test framework identified yet
- Should verify responsive design across devices
- Validate sensor data accuracy and real-time updates
- Ensure accessibility for field use conditions