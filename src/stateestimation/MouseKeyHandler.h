/*
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * 
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
 
 
#pragma once
#include "cvd/image.h"

class MouseKeyHandler
{
public:
	// default constructors
	inline MouseKeyHandler() {};
	inline ~MouseKeyHandler(void) {};
	virtual inline void on_key_down(int key) {};
	virtual inline void on_mouse_move(CVD::ImageRef where, int state) {};
	virtual inline void on_mouse_down(CVD::ImageRef where, int state, int button) {};
	virtual inline void on_event(int event) {};
};
