CFLAGS = -O2 -DU765_EXPORTS -Iinclude
LDFLAGS = -shared

all:
	@echo "Specify a platform target: linux or windows"

linux: libfdc765.so

libfdc765.so: src/fdc765.c include/fdc765.h
	$(CC) $(CFLAGS) $(LDFLAGS) -fPIC -o $@ $<

windows: fdc765.dll

fdc765.dll: src/fdc765.c include/fdc765.h
	$(CC) $(CFLAGS) -D_CRT_SECURE_NO_WARNINGS $(LDFLAGS) -o fdc765.dll $<

clean:
	rm -f libfdc765.so fdc765.dll fdc765.exp fdc765.lib
