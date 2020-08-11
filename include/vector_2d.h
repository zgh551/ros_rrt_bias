/*
 * vector_2d.h
 *
 *  Created on: January 2 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vector_2d.h                         COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 2 2019      Initial Version                  */
/*****************************************************************************/

#ifndef MATH_VECTOR_2D_H_
#define MATH_VECTOR_2D_H_

#include "math.h"

class Vector2d {
public:
	Vector2d();
	Vector2d(const float x,const float y):_x(x),_y(y){}
	virtual ~Vector2d();

	//鐭㈤噺妯￠暱
	float Length(void)const;
	//鐭㈤噺妯￠暱鐨勫钩鏂�
	float LengthSquare(void)const;
	// 鐭㈤噺瑙掑害
	float Angle(void)const;
	//姹傝繛涓煝閲忎箣闂寸殑璺濈
	float DistanceTo(const Vector2d &other)const;

	// 鍚戦噺鍙夌Н axb
	float CrossProduct(const Vector2d&other) const;
	// 鍚戦噺鍐呯Н a.b
	float InnerProduct(const Vector2d&other) const;

	//鐭㈤噺鏃嬭浆锛屾棆杞搴﹂�嗘椂閽堜负姝ｏ紝椤烘椂閽堜负璐�
	Vector2d rotate(const float angle) const;
	//鐭㈤噺姝ｄ氦鐐硅绠�
	Vector2d Orthogonal(const float angle) const;

	// 鑸悜瑙掍负angle鐨勫綊涓�鍖栧悜閲�
	Vector2d Normalize(const float angle) const;

	//杩愮畻绗﹂噸杞�
	Vector2d operator+(const Vector2d &other) const;
	Vector2d operator-(const Vector2d &other) const;
	Vector2d operator*(const double ratio) const;

	float getX();
	void  setX(float value);

	float getY();
	void  setY(float value);

protected:
	float _x;
	float _y;
};

#endif /* MATH_VECTOR_2D_H_ */
