aticle.ps: *.tex references.bib
	pdflatex thesis
	bibtex thesis
	pdflatex thesis
	pdflatex  thesis
	#dvips -t letter -Ppdf -G0 -o thesis.ps thesis.dvi
	#ps2pdf14 -dPDFSETTINGS=/prepress thesis.ps
clean:
	rm -f *.lof *.lot *.toc *.brf *.out *.idx *.aux *.bbl *.blg *.log thesis.dvi thesis.ps thesis.pdf
