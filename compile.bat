@echo off
REM Compile LaTeX document with bibliography in nonstop mode

pdflatex -interaction=nonstopmode main.tex
biber main
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex

echo.
echo Build finished.
exit
