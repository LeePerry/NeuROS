SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

html: Makefile
	plantuml ./plantuml/*.txt
	sphinx-apidoc -o source ..
	sphinx-apidoc -o source ../src
	sphinx-apidoc -o source ../src/neuros/neuros
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

pdf:
	mkdir -p $(BUILDDIR)/pdf
	wkhtmltopdf -T 20 -B 20 --enable-local-file-access $(BUILDDIR)/html/index.html $(BUILDDIR)/pdf/index.pdf
	wkhtmltopdf -T 20 -B 20 --enable-local-file-access $(BUILDDIR)/html/modules.html $(BUILDDIR)/pdf/modules.pdf
	wkhtmltopdf -T 20 -B 20 --enable-local-file-access $(BUILDDIR)/html/src.html $(BUILDDIR)/pdf/src.pdf
	wkhtmltopdf -T 20 -B 20 --enable-local-file-access $(BUILDDIR)/html/neuros.html $(BUILDDIR)/pdf/neuros.pdf
