aticle.ps: *.tex references.bib
	pdflatex main
	bibtex main
	pdflatex main
	pdflatex  main
	#dvips -t letter -Ppdf -G0 -o main.ps main.dvi
	#ps2pdf14 -dPDFSETTINGS=/prepress main.ps
clean:
	rm -f *.aux *.bbl *.blg *.log main.dvi main.ps main.pdf
