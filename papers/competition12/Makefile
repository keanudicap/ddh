FICHIER = main

FINAL   = final
TEXFILES = $(wildcard *.tex) $(wildcard images/*.tex)
BIBFILE  = $(wildcard *.bib)
FIGFILES = $(wildcard images/*.fig)
EPSFILES = $(patsubst %.fig,%.eps,$(FIGFILES))

TMPFILES  = $(wildcard *.aux) $(wildcard *.bbl) $(wildcard *.ps) $(wildcard *.dvi) $(wildcard *.blg) $(wildcard *.toc) $(wildcard *.lof) $(wildcard *.log) $(wildcard *.lot) $(wildcard *.pdf) 
BACKFILES = $(wildcard *.bak) $(wildcard */*.bak) $(wildcard *~) $(wildcard */*~) $(wildcard *.tar.gz)
TRUCFILES = $(wildcard _*)

pdf: $(FINAL).pdf

view: pdf
	xpdf $(FINAL).pdf &

$(FINAL).pdf: $(FICHIER).pdf
	cp $(FICHIER).pdf $(FINAL).pdf

%.pdf: %.ps
	ps2pdf $<

%.ps:  %.dvi
	dvips -Pcmz -Pamz -o $@ $< > /dev/null

%.eps: %.fig
	fig2dev -L eps $< $@

%.dvi: _epsfiles $(TEXFILES)
	latex $(FICHIER)
	if grep -q 'citation' $(FICHIER).aux; then if grep -q 'bibdata' $(FICHIER).aux; then bibtex $(FICHIER); fi else rm -f $(FICHIER).bbl; fi
	latex $(FICHIER) > /dev/null
	latex $(FICHIER) > /dev/null

_epsfiles: $(EPSFILES)
	@echo $(EPSFILES) > _epsfiles

clean:
	-rm -f $(EPSFILES) $(BACKFILES)
	-rm -f $(TMPFILES)
	-rm -f $(TRUCFILES)

tar: clean
	perl -e '$$line=`date +"\%y-\%m-\%d"`;chomp($$line);`tar cvf $(FICHIER)-$$line.tar *`;`gzip $(FICHIER)-$$line.tar`;';

tardir: clean
	perl -e '$$pwd = `pwd`; my($$path,$$dir) = $$pwd =~ m/(.*\/)(.*)$$/; $$line=`date +"\%y-\%m-\%d"`;chomp($$line);`cd .. ; tar cvf $$dir-$$line.tar $$dir ; gzip $$dir-$$line.tar ; mv $$dir-$$line.tar.gz $$dir`;';

pwd:
	perl -e '$$pwd = `pwd`; print $$pwd;'

touch:
	@touch $(FICHIER).tex
	@echo You touch my ``$(FICHIER)''.

open:
#	@kile $(FICHIER).tex &
#	@texmaker $(FICHIER).tex &
	@emacs $(FICHIER).tex &



