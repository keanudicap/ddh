##############################################################################
#
# NOTES:
# 
# 	- just typing 'make' will run latex/dvips/ps2pdf to produce a PDF.
#
# 	- Sometimes, when cross-references change, you need to run 'latex' again.
# 	Typing 'make' won't work because the pdf will be up to date.  To get
# 	around this problem, just type 'make again' which will force latex to 
# 	compile your ahastar.  You'll need to type 'make' to run dvips/ps2pdf.
#
###############################################################################

LATEX=pdflatex
DVIPS=dvips
PS2PDF=ps2pdf14
BIBTEX=bibtex

LATEX_OPTS=-interaction=nonstopmode -halt-on-error
DVIPS_OPTS=-Ppdf -t letter
PS2PDF_OPTS=-dPDFSETTINGS=/prepress -dSubsetFonts=true -dEmbedAllFonts=true
NUM=""

RERUN = "(There were undefined references|Rerun to get (cross-references|the bars) right)"
RERUNBIB = "No file.*\.bbl|Citation.*undefined"


.PHONY: all clean


all: ahastar.tex references.bbl again embeded

ahastar: ahastar.pdf *.tex

ahastar.pdf :
	$(LATEX) $(LATEX_OPTS) ahastar.tex

references.bbl: references.bib
	if [ -n ahastar.aux ]; \
	then \
		$(LATEX) $(LATEX_OPTS) ahastar.tex;\
	fi
	$(BIBTEX) ahastar

again:
	$(LATEX) $(LATEX_OPTS) ahastar.tex


clean:
	+rm -fv ahastar.{dvi,ps,pdf,aux,log,bbl,blg}

embeded:
	 ps2pdf -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress ahastar.pdf embeded-ahastar.pdf
