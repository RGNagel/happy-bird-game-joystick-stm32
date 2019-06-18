/*
 * happy_bird.h
 *
 *  Created on: 4 de jun de 2019
 *      Author: rgnagel
 */

#ifndef HAPPY_BIRD_H_
#define HAPPY_BIRD_H_

#include "PRNG_LFSR.h"
#include "NOKIA5110_fb.h"

#define true 1
#define TRUE 1
#define false 0
#define FALSE 0

typedef unsigned int bool;

#define MAX_X 83
#define MAX_Y 47
#define UPPER_BOUND 2200
#define LOWER_BOUND 1800


#define IS_WITHIN(x, x1, x2) (((x) >= (x1)) && ((x) <= (x2)))

enum fsm {
	STARTING,
	PLAYING,
	GAME_OVER,
};

struct control {
	uint32_t gems_collected;
	uint32_t row;
	uint32_t superspeed;
	uint32_t obstacle_step;
	uint32_t bird_step;
	bool play_again;
};

static enum fsm hb_fsm = STARTING;

static struct control hb_control = {
		.gems_collected = 0,
		.row = 0,
		.superspeed = 0,
		.obstacle_step = 1,
		.bird_step = 1,
		.play_again = true,
};

/*
 * all bitmap pictures must be 48x84 (width x height) or less
 * if escreve2fb() is used the picture must be 48x84 or bigger
 * i.e. the converted vector must contain 504 elements or more
 */

// 25 x 18 (width x height)
static const struct figura_t hb_bird_fig = {
		.largura = 25,
		.altura = 18,
		.pixels = {
				0x80, 0x60, 0x20, 0x20, 0x20, 0x38, 0x24, 0x64, 0xC6, 0x03, 0x01, 0x01, 0x01, 0xF9, 0x9D, 0x0F,
				0x07, 0x46, 0xE6, 0x4C, 0x18, 0xF0, 0x00, 0x00, 0x00, 0x03, 0x04, 0x08, 0x10, 0x10, 0x10, 0x70,
				0x98, 0x8F, 0x80, 0x00, 0x00, 0x00, 0x10, 0x01, 0x4B, 0x92, 0x92, 0x92, 0x92, 0x92, 0x93, 0xF2,
				0x12, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		}
};
static const struct figura_t hb_black_little_bird_fig = {
		.largura = 11,
		.altura = 11,
		.pixels = {
				0x70, 0xFC, 0xFE, 0xFE, 0xFF, 0x77, 0x5F, 0x56, 0xDE, 0xDC, 0x50, 0x00, 0x01, 0x03, 0x03
		}
};

static struct pontos_t hb_bird_pts;
static struct figura_t *bird = &hb_black_little_bird_fig;

static const struct figura_t hb_obstacle_fig = {
		.largura = 10,
		.altura = 47,
		.pixels = {
				0x84, 0xCE, 0xFF, 0xCE, 0x84, 0x84, 0xCE, 0xFF, 0xCE, 0x84, 0x10, 0x39, 0xFF, 0x39, 0x10, 0x10,
				0x39, 0xFF, 0x39, 0x10, 0x42, 0xE7, 0xFF, 0xE7, 0x42, 0x42, 0xE7, 0xFF, 0xE7, 0x42, 0x08, 0x9C,
				0xFF, 0x9C, 0x08, 0x08, 0x9C, 0xFF, 0x9C, 0x08, 0x21, 0x73, 0xFF, 0x73, 0x21, 0x21, 0x73, 0xFF,
				0x73, 0x21, 0x04, 0x4E, 0x7F, 0x4E, 0x04, 0x04, 0x4E, 0x7F,
		}
};

static struct pontos_t hb_obstacle_pts;
static struct figura_t *obstacle = &hb_obstacle_fig;

static const struct figura_t hb_gem_1 = {
		.largura = 15,
		.altura = 15,
		.pixels = {
				0xC0, 0xF0, 0xFC, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFC, 0xFC, 0xF0, 0xC0, 0x01,
				0x07, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0x1F,
		}
};
static const struct figura_t hb_gem_3 = {
		.largura = 7,
		.altura = 7,
		.pixels = {
				0x49, 0x2A, 0x1C, 0x77, 0x1C, 0x2A
		}
};

static struct figura_t *gem = &hb_gem_3;

static const unsigned char hb_opening [] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x3F,
		0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
		0x3F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0x01, 0x01,
		0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xFC, 0xFC, 0xFC, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xC1, 0xC1, 0xC1, 0xC1, 0x01, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0E, 0x0E, 0x0E, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x70, 0x70, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x70,
		0x70, 0x70, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
		0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
		0x03, 0x00, 0x38, 0x38, 0x38, 0x38, 0x00, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83,
		0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
		0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

/* get a random point from which we will draw a empty rectangle for the bird to go through */
static inline uint32_t get_rand_y_obstacle(void)
{
	return prng_LFSR() % (MAX_Y - bird->altura);
}

static inline uint32_t get_rand_x_gem(void)
{
	return prng_LFSR() % (MAX_X - gem->largura);
}
static inline uint32_t get_rand_y_gem(void)
{
	return prng_LFSR() % (MAX_Y - gem->altura);
}

/* prop: 0: clean, otherwise draw */
void desenha_retangulo_preenchido(struct pontos_t *pts, uint32_t prop);

#define apaga_retangulo_preenchido(pts) desenha_retangulo_preenchido(pts, 0)

void apaga_fig(struct pontos_t *pts, struct figura_t *fig);

bool overlaps(const struct pontos_t *p1, const struct figura_t *f1,
		      const struct pontos_t *p2, const struct figura_t *f2);

#endif /* HAPPY_BIRD_H_ */
