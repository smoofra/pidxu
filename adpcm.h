/* Functions from adpcm.c */
unsigned char LinearToMuLawSample(short sample);
unsigned char LinearLongToMuLawSample(long sample);
unsigned char LinearToALawSample(short sample);
extern short MuLawDecompressTable[256];
extern short ALawDecompressTable[256];
/* end of adpcm.c */
