all: article short
article: *.tex references.bib
	pdflatex article
	bibtex article
	pdflatex article
	pdflatex  article
short: references.bib
	pdflatex short_article
	bibtex short_article
	pdflatex short_article
	pdflatex  short_article
clean:
	rm -f *.aux *.bbl *.blg *.log *.dvi *.ps *.pdf 
