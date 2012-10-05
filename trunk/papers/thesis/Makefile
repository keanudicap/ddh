
all: draft

full: *.tex references.bib
	pdflatex thesis
	bibtex thesis
	pdflatex thesis
	pdflatex thesis

draft: *.tex references.bib
	pdflatex draft
	bibtex draft
	pdflatex draft
	pdflatex draft

clean:
	rm -f *.lof *.lot *.toc *.brf *.out *.idx *.aux *.bbl *.blg *.log thesis.pdf draft.pdf
