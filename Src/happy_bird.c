/*
 * happy_bird.c
 *
 *  Created on: 11 de jun de 2019
 *      Author: rgnagel
 */

#include "happy_bird.h"
#include "PRNG_LFSR.h"


/* prop: 0: clean, otherwise draw */
void desenha_retangulo_preenchido(struct pontos_t *pts, uint32_t prop)
{
	struct pontos_t line;
	line.y1 = pts->y1;
	line.y2 = pts->y2;

	for (uint32_t dx = pts->x1; dx <= pts->x2; dx++) {
		line.x1 = dx;
		line.x2 = dx;
		desenha_linha(&line, prop);
	}
}
void apaga_fig(struct pontos_t *pts, struct figura_t *fig)
{
	struct pontos_t rect;
	rect.x1 = pts->x1;
	rect.y1 = pts->y1;
	if (pts->y2 == 0 || pts->x2 == 0) {
		rect.x2 = pts->x1 + fig->largura;
		rect.y2 = pts->y1 + fig->altura;
	}
	else {
		rect.x2 = pts->x2;
		rect.y2 = pts->y2;
	}
	desenha_retangulo_preenchido(&rect, 0);
}


bool overlaps(const struct pontos_t *p1, const struct figura_t *f1,
		      const struct pontos_t *p2, const struct figura_t *f2)
{
	uint32_t p1x2, p1y2, p2x2, p2y2;
	bool vx1 = false, vx2 = false, vy1 = false, vy2 = false;

	// if x2 or y2 is zero then use figure dimension
	p1x2 = (p1->x2) ? p1->x2 : (p1->x1 + f1->largura -2);
	p1y2 = (p1->y2) ? p1->y2 : (p1->y1 + f1->altura -2);
	p2x2 = (p2->x2) ? p2->x2 : (p2->x1 + f2->largura -2);
	p2y2 = (p2->y2) ? p2->y2 : (p2->y1 + f2->altura -2);

	// x1
	if (IS_WITHIN(p1->x1, p2->x1, p2x2))
		vx1 = true;
	// x2
	if (IS_WITHIN(p1x2, p2->x1, p2x2))
		vx2 = true;
	// y1
	if (IS_WITHIN(p1->y1, p2->y1, p2y2))
		vy1 = true;
	// y2
	if (IS_WITHIN(p1y2, p2->y1, p2y2))
		vy2 = true;

	// here each figure is a square. we need to check the four vertices of the figure: x1, x2, y1, y2
	if ((vx1 && vy1) || (vx1 && vy2) || (vx2 && vy1) || (vx2 && vy2))
		return true;
	else
		return false;
}
