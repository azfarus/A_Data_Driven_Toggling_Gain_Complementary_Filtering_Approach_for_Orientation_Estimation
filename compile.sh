#!/bin/bash
# Usage: ./build.sh main.tex

# Exit immediately if a command fails
set -e

# Get filename without extension
filename=$(basename "$1" .tex)

# 1st LaTeX pass
pdflatex -interaction=nonstopmode "$filename.tex"

# Bibliography
biber "$filename"

# 2nd and 3rd LaTeX passes (to resolve references)
pdflatex -interaction=nonstopmode "$filename.tex"
pdflatex -interaction=nonstopmode "$filename.tex"

echo "✅ Build complete: $filename.pdf"