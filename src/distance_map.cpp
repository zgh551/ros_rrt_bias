#include "../include/distance_map.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>

map::DistanceMap::DistanceMap()
{
    _eight_neighbors_mask = new Vector2d[9];

    _eight_neighbors_mask[0].setX(0.0);
    _eight_neighbors_mask[0].setY(0.0);

    _eight_neighbors_mask[1].setX(1.0);
    _eight_neighbors_mask[1].setY(0.0);

    _eight_neighbors_mask[2].setX(1.0);
    _eight_neighbors_mask[2].setY(1.0);

    _eight_neighbors_mask[3].setX(0.0);
    _eight_neighbors_mask[3].setY(1.0);

    _eight_neighbors_mask[4].setX(-1.0);
    _eight_neighbors_mask[4].setY( 1.0);

    _eight_neighbors_mask[5].setX(-1.0);
    _eight_neighbors_mask[5].setY( 0.0);

    _eight_neighbors_mask[6].setX(-1.0);
    _eight_neighbors_mask[6].setY(-1.0);

    _eight_neighbors_mask[7].setX( 0.0);
    _eight_neighbors_mask[7].setY(-1.0);

    _eight_neighbors_mask[8].setX( 1.0);
    _eight_neighbors_mask[8].setY(-1.0);

    // initial the mask M1
    _M1.clear();
    _M1.push_back(_eight_neighbors_mask[0]);
    for (uint8_t i = 5; i < 9; i++)
    {
        _M1.push_back(_eight_neighbors_mask[i]);
    }

    // initial the mask M2
    _M2.clear();
    for (uint8_t i = 0; i < 2; i++)
    {
        _M2.push_back(_eight_neighbors_mask[i]);
    }

    // initial the mask M3
    _M3.clear();
    for (uint8_t i = 0; i < 5; i++)
    {
        _M3.push_back(_eight_neighbors_mask[i]);
    }

    // initial the mask M4
    _M4.clear();
    _M4.push_back(_eight_neighbors_mask[0]);
    _M4.push_back(_eight_neighbors_mask[5]);

}

map::DistanceMap::~DistanceMap()
{
    delete[] _eight_neighbors_mask;    
}

double map::DistanceMap::CalculateThreshold(uint16_t n)
{
    double temp_a = sqrt(8 * n - 11);
    return sqrt(2 / (4 - (sqrt(16 * n * n - 40 * n + 22 - 2 * temp_a) / (n - 1)))) - sqrt((pow(temp_a - 1, 2) + 4) / 16);
}

int8_t map::DistanceMap::DistanceMapUpdate(uint16_t width, uint16_t height, int8_t *input_map, double *output_map)
{

    Vector2d *init_map = new Vector2d[ width * height];
    double threshold = 0;//CalculateThreshold(std::max(width, height));
    ROS_INFO("the threshold is:%f,the size of N is:%d", threshold, width * height);

    for (uint16_t j = 0; j < height; j++)
    {
        for(uint16_t i = 0; i < width; i++)
        {
            init_map[i + j * width] =   input_map[i + j * width] > 0 ? 
                                        Vector2d(0.0, 0.0) : 
                                        Vector2d(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
        }
    }

    int16_t index_x = 0, index_y = 0;
    double min_distance = 0.0;
    std::vector<std::pair<double,uint8_t>> d;
    for (uint16_t j = 0; j < height; j++)
    {
        for (uint16_t i = 0; i < width; i++)
        {
            /// begin the mask M1 ///
            d.clear();
            for (uint8_t k = 0; k < _M1.size(); k++)
            {
                index_x = i + static_cast<int16_t>(_M1[k].getX());
                index_y = j + static_cast<int16_t>(_M1[k].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    if ( (init_map[index_x + index_y * width].getX() < (std::numeric_limits<double>::max() - 2))
                      && (init_map[index_x + index_y * width].getY() < (std::numeric_limits<double>::max() - 2))
                    )
                    {
                        d.push_back( std::pair<double, uint8_t>((init_map[index_x + index_y * width] + _M1[k]).Length(), k) );
                    }
                    else
                    {
                       // ROS_INFO("distance is infine");
                    }
                }
                else
                {
                   // ROS_INFO("index out the boundary");
                }
            } // end k

            // get the min distance and update the map
            if(d.size() == 0)
            {
                //ROS_INFO("no valid distance!");
            }
            else if (1 == d.size())
            {
                min_distance = d[0].first;
                index_x = i + static_cast<int16_t>(_M1[d[0].second].getX());
                index_y = j + static_cast<int16_t>(_M1[d[0].second].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    init_map[i + j * width] = init_map[index_x + index_y * width] + _M1[d[0].second]; 
                }
                else
                {
                    ROS_INFO("sencond out index ERR!!!");
                }
            }
            else
            {
                std::sort(d.begin(), d.end(), [](const std::pair<double, uint8_t> &i, const std::pair<double, uint8_t> &j){
                    return i.first < j.first;
                });
                min_distance = d[0].first;

                for (std::vector<std::pair<double, uint8_t>>::iterator iter = d.begin(); iter != d.end(); iter++)
                {
                    if ((*iter).first <= (min_distance + threshold))
                    {
                        index_x = i + static_cast<int16_t>(_M1[(*iter).second].getX());
                        index_y = j + static_cast<int16_t>(_M1[(*iter).second].getY());
                        if ((index_x >= 0) && (index_x < width) 
                         && (index_y >= 0) && (index_y < height))
                        {
                            init_map[i + j * width] = init_map[index_x + index_y * width] + _M1[(*iter).second]; 
                        }
                        else
                        {
                            ROS_INFO("sencond out index ERR!!!");
                        }
                    }
                    else
                    {
                        //ROS_INFO("M1:the distance is too big! disatnce:%f, min_distance:%f", (*iter).first, min_distance);
                    }
                }
            }
            /// end of mask M1/// 
        } // end i

        for (int16_t i = width - 1; i >= 0; i--)
        {
            /// begin the mask M2 ///
            d.clear();
            for (uint8_t k = 0; k < _M2.size(); k++)
            {
                index_x = i + static_cast<int16_t>(_M2[k].getX());
                index_y = j + static_cast<int16_t>(_M2[k].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    if ( (init_map[index_x + index_y * width].getX() < (std::numeric_limits<double>::max() - 2))
                      && (init_map[index_x + index_y * width].getY() < (std::numeric_limits<double>::max() - 2))
                    )
                    {
                        d.push_back( std::pair<double, uint8_t>((init_map[index_x + index_y * width] + _M2[k]).Length(), k) );
                    }
                    else
                    {
                        //ROS_INFO("distance is infine");
                    }
                }
                else
                {
                    //ROS_INFO("index out the boundary");
                }
            } // end k

            // get the min distance and update the map
            if (0 == d.size())
            {
                //ROS_INFO("no valid distance!");
            }
            else if (1 == d.size())
            {
                min_distance = d[0].first;
                index_x = i + static_cast<int16_t>(_M2[d[0].second].getX());
                index_y = j + static_cast<int16_t>(_M2[d[0].second].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    init_map[i + j * width] = init_map[index_x + index_y * width] + _M2[d[0].second]; 
                }
                else
                {
                    ROS_INFO("sencond out index ERR!!!");
                }
            }
            else
            {
                std::sort(d.begin(), d.end(), [](const std::pair<double, uint8_t> &i, const std::pair<double, uint8_t> &j){
                    return i.first < j.first;
                });
                min_distance = d[0].first;

                for (std::vector<std::pair<double, uint8_t>>::iterator iter = d.begin(); iter != d.end(); iter++)
                {
                    if ((*iter).first <= (min_distance + threshold))
                    {
                        index_x = i + static_cast<int16_t>(_M2[(*iter).second].getX());
                        index_y = j + static_cast<int16_t>(_M2[(*iter).second].getY());
                        if ((index_x >= 0) && (index_x < width) 
                         && (index_y >= 0) && (index_y < height))
                        {
                            init_map[i + j * width] = init_map[index_x + index_y * width] + _M2[(*iter).second]; 
                        }
                        else
                        {
                            ROS_INFO("sencond out index ERR!!!");
                        }
                    }
                    else
                    {
                  //      ROS_INFO("M2:the distance is too big! disatnce:%f, min_distance:%f", (*iter).first, min_distance);
                    }
                }
            }
            /// end of mask M2/// 
        }// end line
    }// end column
    /*
    for (uint16_t j = 0; j < height; j++)
    {
        for(uint16_t i = 0; i < width; i++)
        {
            std::cout << "(" << init_map[i + j * width].getX() << "," << init_map[i + j * width].getY() << ") ";
        }
        std::cout << "\r\n";
    }
    */

    for (int16_t j = height - 1; j >= 0; j--)
    {
        for (int16_t i = width - 1; i >= 0; i--)
        {
            /// begin the mask M3 ///
            d.clear();
            for (uint8_t k = 0; k < _M3.size(); k++)
            {
                index_x = i + static_cast<int16_t>(_M3[k].getX());
                index_y = j + static_cast<int16_t>(_M3[k].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    if ( (init_map[index_x + index_y * width].getX() < (std::numeric_limits<double>::max() - 2))
                      && (init_map[index_x + index_y * width].getY() < (std::numeric_limits<double>::max() - 2))
                    )
                    {
                        d.push_back( std::pair<double, uint8_t>((init_map[index_x + index_y * width] + _M3[k]).Length(), k) );
                    }
                    else
                    {
                    //    ROS_INFO("distance is infine");
                    }
                }
                else
                {
                   // ROS_INFO("index out the boundary");
                }
            } // end k

            // get the min distance and update the map
            if(d.size() == 0)
            {
           //     ROS_INFO("no valid distance!");
            }
            else if (1 == d.size())
            {
                min_distance = d[0].first;
                index_x = i + static_cast<int16_t>(_M3[d[0].second].getX());
                index_y = j + static_cast<int16_t>(_M3[d[0].second].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    init_map[i + j * width] = init_map[index_x + index_y * width] + _M3[d[0].second]; 
                }
                else
                {
                    ROS_INFO("sencond out index ERR!!!");
                }
            }
            else
            {
                std::sort(d.begin(), d.end(), [](const std::pair<double, uint8_t> &i, const std::pair<double, uint8_t> &j){
                    return i.first < j.first;
                });
                min_distance = d[0].first;

                for (std::vector<std::pair<double, uint8_t>>::iterator iter = d.begin(); iter != d.end(); iter++)
                {
                    if ((*iter).first <= (min_distance + threshold))
                    {
                        index_x = i + static_cast<int16_t>(_M3[(*iter).second].getX());
                        index_y = j + static_cast<int16_t>(_M3[(*iter).second].getY());
                        if ((index_x >= 0) && (index_x < width) 
                         && (index_y >= 0) && (index_y < height))
                        {
                            init_map[i + j * width] = init_map[index_x + index_y * width] + _M3[(*iter).second]; 
                        }
                        else
                        {
                            ROS_INFO("sencond out index ERR!!!");
                        }
                    }
                    else
                    {
             //           ROS_INFO("M3:the distance is too big! disatnce:%f, min_distance:%f", (*iter).first, min_distance);
                    }
                }
            }
            /// end of mask M3/// 
        } // end i

        for (uint16_t i = 0; i < width; i++)
        {
            /// begin the mask M4 ///
            d.clear();
            for (uint8_t k = 0; k < _M4.size(); k++)
            {
                index_x = i + static_cast<int16_t>(_M4[k].getX());
                index_y = j + static_cast<int16_t>(_M4[k].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    if ( (init_map[index_x + index_y * width].getX() < (std::numeric_limits<double>::max() - 2))
                      && (init_map[index_x + index_y * width].getY() < (std::numeric_limits<double>::max() - 2))
                    )
                    {
                        d.push_back( std::pair<double, uint8_t>((init_map[index_x + index_y * width] + _M4[k]).Length(), k) );
                    }
                    else
                    {
               //         ROS_INFO("distance is infine");
                    }
                }
                else
                {
                 //   ROS_INFO("index out the boundary");
                }
            } // end k

            // get the min distance and update the map
            if (0 == d.size())
            {
                //ROS_INFO("no valid distance!");
            }
            else if (1 == d.size())
            {
                min_distance = d[0].first;
                index_x = i + static_cast<int16_t>(_M4[d[0].second].getX());
                index_y = j + static_cast<int16_t>(_M4[d[0].second].getY());
                if ((index_x >= 0) && (index_x < width) 
                 && (index_y >= 0) && (index_y < height))
                {
                    init_map[i + j * width] = init_map[index_x + index_y * width] + _M4[d[0].second]; 
                }
                else
                {
                    ROS_INFO("sencond out index ERR!!!");
                }
            }
            else
            {
                std::sort(d.begin(), d.end(), [](const std::pair<double, uint8_t> &i, const std::pair<double, uint8_t> &j){
                    return i.first < j.first;
                });
                min_distance = d[0].first;

                for (std::vector<std::pair<double, uint8_t>>::iterator iter = d.begin(); iter != d.end(); iter++)
                {
                    if ((*iter).first <= (min_distance + threshold))
                    {
                        index_x = i + static_cast<int16_t>(_M4[(*iter).second].getX());
                        index_y = j + static_cast<int16_t>(_M4[(*iter).second].getY());
                        if ((index_x >= 0) && (index_x < width) 
                         && (index_y >= 0) && (index_y < height))
                        {
                            init_map[i + j * width] = init_map[index_x + index_y * width] + _M4[(*iter).second]; 
                        }
                        else
                        {
                            ROS_INFO("sencond out index ERR!!!");
                        }
                    }
                    else
                    {
                  //      ROS_INFO("M4:the distance is too big! disatnce:%f, min_distance:%f", (*iter).first, min_distance);
                    }
                }
            }
            /// end of mask M4/// 
            output_map[i + j * width] = min_distance; 
            //std::cout << min_distance << " ";
        }// end line
        //std::cout << "\r\n";
    }// end column
    /*
    for (uint16_t j = 0; j < height; j++)
    {
        for(uint16_t i = 0; i < width; i++)
        {
            //output_map[i + j * width] = init_map[i + j * width].Length();
            std::cout << output_map[i + j * width] << " ";
        }
        std::cout << "\r\n";
    }
    */
    delete[] init_map;
    return 0;
}
