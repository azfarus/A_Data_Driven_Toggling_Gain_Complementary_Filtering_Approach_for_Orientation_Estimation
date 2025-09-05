@echo off
REM Compile LaTeX document with bibliography and SVG images in nonstop mode

pdflatex --shell-escape -interaction=nonstopmode main.tex
biber main
pdflatex --shell-escape -interaction=nonstopmode main.tex
pdflatex --shell-escape -interaction=nonstopmode main.tex

echo.
echo Build finished.
exit
