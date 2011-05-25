poster.ps: *.tex 
	pdflatex poster.tex
#	bibtex poster
	pdflatex  poster.tex
	#dvips -t letter -Ppdf -G0 -o poster.ps poster.dvi
	#ps2pdf14 -dPDFSETTINGS=/prepress poster.ps
clean:
	rm -f *.aux *.bbl *.blg *.log poster.dvi poster.ps poster.pdf
