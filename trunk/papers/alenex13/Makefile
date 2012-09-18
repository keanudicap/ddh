aticle.ps: *.tex references.bib
	pdflatex paper
	bibtex paper
	pdflatex paper
	pdflatex paper
	#dvips -t letter -Ppdf -G0 -o paper.ps paper.dvi
	#ps2pdf14 -dPDFSETTINGS=/prepress paper.ps
clean:
	rm -f *.aux *.bbl *.blg *.log paper.dvi paper.ps paper.pdf
