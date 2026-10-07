p='support/minimig/minimig_a4091.cpp'; s=open(p).read()
a="""	// Safe path: unmapped / window-straddling range.
	for (dma_addr_t i = 0; i < len; i++) {"""
assert a in s
s=s.replace(a,"""	// Chip / slow / Zorro II RAM: not HPS memory. A combined core lends the
	// A2091's DMA engine (chipdma_arb) for it.
	if (a2091_dma_rw((uint32_t)addr, p, (uint32_t)len, dir != DMA_DIRECTION_TO_DEVICE))
		return 0;

	// Safe path: unmapped / window-straddling range.
	for (dma_addr_t i = 0; i < len; i++) {""",1)
open(p,'w').write(s); print("ok")
