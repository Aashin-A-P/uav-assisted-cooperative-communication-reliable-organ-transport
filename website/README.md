# UAV Relay Website

Single-page React frontend for your MATLAB DRL UAV simulation UI.

## Layout

- Top: Control Center
- Bottom Left: Source Hospital
- Bottom Right: Destination Hospital

## Run

```bash
cd website
npm install
npm run dev
```

## Notes

- Current version is styling-first with placeholder telemetry values.
- Organ matching reads Excel from `public/data/organ_available.xlsx`.
- Replace the "MATLAB Animation Feed" area in `src/App.jsx` with:
  - a video/canvas stream exported from MATLAB, or
  - replay logic from MATLAB-generated JSON trajectory data.
