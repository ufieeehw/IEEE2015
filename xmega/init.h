#ifndef INIT_H_
#define INIT_H_

#ifndef F_CPU
#define F_CPU 32000000
#endif


void init();

void init_clocks();
void init_interrupts();
void init_modules();

#endif /* INIT_H_ */