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

double Vector2d::Length(void)const
{
    return hypot(this->_x,this->_y);
}

double Vector2d::LengthSquare(void)const
{
	return this->_x * this->_x + this->_y * this->_y;
}

double Vector2d::Angle(void)const
{
    return atan2(this->_y , this->_x);
}

double Vector2d::DistanceTo(const Vector2d &other)const
{
    return hypot(this->_x - other._x , this->_y - other._y);
}

// 向量叉积 axb
double Vector2d::CrossProduct(const Vector2d&other) const
{
    return this->_x * other._y - this->_y * other._x;
}

// 向量内积 a.b
double Vector2d::InnerProduct(const Vector2d&other) const
{
    return this->_x * other._x + this->_y * other._y;
}

Vector2d Vector2d::rotate(const double angle) const
{
    double s_value,c_value;
    s_value = sin(angle);
    c_value = cos(angle);
	return Vector2d(this->_x * c_value - this->_y * s_value,
			        this->_x * s_value + this->_y * c_value);
}

Vector2d Vector2d::Orthogonal(const double angle) const
{
    double s_value,c_value;
    s_value = sin(angle);
    c_value = cos(angle);
	return Vector2d(this->_x * c_value * c_value + this->_y * s_value * c_value,
			        this->_x * s_value * c_value + this->_y * s_value * s_value);
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

