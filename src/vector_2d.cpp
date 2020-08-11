/*
 * vector_2d.cpp
 *
 *  Created on: January 2 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vector_2d.cpp                       COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 2 2019      Initial Version                  */
/*****************************************************************************/
#include "../include/vector_2d.h"

Vector2d::Vector2d() {

}

Vector2d::~Vector2d() {

}

float Vector2d::Length(void)const
{
	return hypotf(this->_x,this->_y);
}

float Vector2d::LengthSquare(void)const
{
	return this->_x * this->_x + this->_y * this->_y;
}

float Vector2d::Angle(void)const
{
	return atan2f(this->_y , this->_x);
}

float Vector2d::DistanceTo(const Vector2d &other) const
{
	return hypotf(this->_x - other._x , this->_y - other._y);
}

// 向量叉积 axb
float Vector2d::CrossProduct(const Vector2d&other) const
{
	return this->_x * other._y - this->_y * other._x;
}

// 向量内积 a.b
float Vector2d::InnerProduct(const Vector2d&other) const
{
	return this->_x * other._x + this->_y * other._y;
}


Vector2d Vector2d::rotate(const float angle) const
{
	float s_value,c_value;
	s_value = sinf(angle);
	c_value = cosf(angle);
	return Vector2d(this->_x * c_value - this->_y * s_value,
			        this->_x * s_value + this->_y * c_value);
}

Vector2d Vector2d::Orthogonal(const float angle) const
{
	float s_value,c_value;
	s_value = sinf(angle);
	c_value = cosf(angle);
	return Vector2d(this->_x * c_value * c_value + this->_y * s_value * c_value,
			        this->_x * s_value * c_value + this->_y * s_value * s_value);
}

// 航向角为angle的归一化向量
Vector2d Vector2d::Normalize(const float angle) const{
//	return Vector2d(cosf(angle),sinf(angle));
}

Vector2d Vector2d::operator+(const Vector2d& other) const
{
	return Vector2d(this->_x + other._x,this->_y + other._y);
}

Vector2d Vector2d::operator-(const Vector2d& other) const
{
	return Vector2d(this->_x - other._x,this->_y - other._y);
}

Vector2d Vector2d::operator*(const double ratio) const
{
	return Vector2d(this->_x * ratio,this->_y * ratio);
}

float Vector2d::getX()           { return  _x;}
void  Vector2d::setX(float value){ _x = value;}

float Vector2d::getY()           { return  _y;}
void  Vector2d::setY(float value){ _y = value;}
