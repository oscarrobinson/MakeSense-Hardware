INSTALL= /usr/local/bin

################

LDLIBS = -lftdi

TARGETS = bbmc bbmc_n bitbang_cbus #usbreset

CFLAGS = -Wall -Wextra #-Werror

all: $(TARGETS)

clean:
	-rm -f $(TARGETS)

install: all
	cp bbmc $(INSTALL)
