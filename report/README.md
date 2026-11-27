# Technical report (HardwareX format)

LaTeX source of the project report, written to the structure of a HardwareX article.

## Compile on Overleaf

1. Zip this `report/` folder (or upload its contents) as a new Overleaf project.
2. Main document: `main.tex`. Compiler: **pdfLaTeX** (Overleaf default). TeX Live 2021 or newer.
3. Recompile; Overleaf runs BibTeX automatically (`references.bib`, style `elsarticle-num`).

Only standard packages are used (TikZ, `siunitx`, `booktabs`, `natbib`, `cleveref`, ...).

The page layout comes from `hxstyle.sty`, which is modelled on HardwareX's typeset article
style: grey header band, left-aligned title, ARTICLE INFO / ABSTRACT block, Charter text,
bold "Fig." captions, blue links and running heads. Publisher logos, the journal masthead,
DOI and copyright lines are deliberately not reproduced, because this is a preprint. The
band text, running heads and article-info column are set near the top of `main.tex`. For an actual
submission, move the sections into the journal's official template (Elsevier `elsarticle`).

Local build (Ubuntu): `latexmk -pdf main.tex` (packages: `texlive-latex-recommended
texlive-latex-extra texlive-pictures texlive-science texlive-fonts-recommended lmodern latexmk`).
All diagrams (architecture, wiring, packet formats, flowcharts, TF tree) are TikZ code inside
the section files, so they can be edited directly.

## Layout

```
main.tex                 preamble, title block, section includes
references.bib           bibliography
sections/                one file per section (00_abstract ... 10_appendix)
figures/                 data plots (PDF), trade-study charts and camera images
data/                    bench telemetry log, segmentation benchmark, SITL flight extracts (sim/), statistics
scripts/                 regenerate every figure/number this report measures itself
```

## Before submission: items left for the authors

All in-text TODOs are resolved. Still open:

- the corresponding author (name and e-mail) in `main.tex`
- a DOI for an archived release (Zenodo/OSF), promised in the specifications table
- adding `LICENSE` files (MIT for software, CERN-OHL-P v2 for hardware) to the repository, to match the licence stated in the report

Real-vehicle results are a 7-minute hover (1.6 m drift, measured against ground marks) and an
autonomous square-path flight without an external reference; trajectory accuracy against ground
truth is still to be measured (Section 9).

## Regenerating the measured results

From the repository root:

```bash
python3 report/scripts/bench_segmentation.py --iters 100   # needs CUDA GPU + ros2_ws/*.pt weights
python3 report/scripts/make_figures.py                      # bench-log plots, ZUPT, dead reckoning
```

`make_figures.py` writes `figures/*.pdf` and `data/bench_stats.txt` (the numbers quoted in
Section 8). `data/bench_telemetry_2026-11-15.csv` is a copy of `raspberry_pi/plotjuggler_live.csv`.

Simulation flight analysis (Section 8.10, figures `sim_*.pdf`, tables in `data/sim/*_table.tex`):

```bash
# 1. once, on the machine that ran the simulation (needs pyulog; reads ~1.7 GB of ULog files)
python3 report/scripts/extract_sim_logs.py \
    --ulog-dir ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-20 --ros-log-dir ~/.ros/log
# 2. anywhere: figures, LaTeX tables and data/sim/sim_stats.txt from the CSVs in data/sim/
python3 report/scripts/make_sim_figures.py
```

`data/sim/` holds the extracted 5 Hz time series (`run_*.csv`), LiDAR maps at Gazebo truth
(`scan_*.csv`) and the per-flight summary (`sim_runs.csv`), so step 2 does not need the raw logs.
