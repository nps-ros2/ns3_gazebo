drawings = \
	nns_simple.png real.png simulated.png simulated_flows.png \
	simulated_motion.png nns_wifi_only.png

.SUFFIXES: .svg .pdf
.SUFFIXES: .svg .png
.SUFFIXES: .m   .svg

.m.svg:
	octave -q $<

.svg.pdf:
	inkscape --without-gui -D --export-pdf=$@ $<

.svg.png:
	inkscape --without-gui -D --export-png=$@ $<

all: $(drawings)

clean:
	rm -f *.pdf *.png

