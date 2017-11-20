%.dot: %
	./visualize.py < $< > $@

%.png: %.dot
	dot -Tpng -o $@ $<

%.svg: %.dot
	dot -Tsvg -o $@ $<

%.2pi-png: %.dot
	twopi -Tpng -o $@ $<
