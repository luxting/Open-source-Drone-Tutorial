#include "route_single_goods.h"
#include <iostream>

using namespace std;

float (*getRoute(int goods_locate))[2] 
{
    int shelf = goods_locate / 10;
    int shelf_sn = goods_locate %10;
    if (shelf == 1) 
    {
        if (shelf_sn == 1 |shelf_sn ==4)
        {
            return route_A1_4;
        }
        else if (shelf_sn == 2 |shelf_sn == 5)
        {
            return route_A2_5;
        }
        else if (shelf_sn == 3 |shelf_sn == 6)
        {
            return route_A3_6;
        }
    } 
    else if (shelf == 2) 
    {
        if (shelf_sn == 1 |shelf_sn ==4)
        {
            return route_B1_4;
        }
        else if (shelf_sn == 2 |shelf_sn == 5)
        {
            return route_B2_5;
        }
        else if (shelf_sn == 3 |shelf_sn == 6)
        {
            return route_B3_6;
        }
    }
    else if (shelf == 3) 
    {
        if (shelf_sn == 1 |shelf_sn ==4)
        {
            return route_C1_4;
        }
        else if (shelf_sn == 2 |shelf_sn == 5)
        {
            return route_C2_5;
        }
        else if (shelf_sn == 3 |shelf_sn == 6)
        {
            return route_C3_6;
        }
    }
    else if (shelf == 4) 
    {
        if (shelf_sn == 1 |shelf_sn ==4)
        {
            return route_D1_4;
        }
        else if (shelf_sn == 2 |shelf_sn == 5)
        {
            return route_D2_5;
        }
        else if (shelf_sn == 3 |shelf_sn == 6)
        {
            return route_D3_6;
        }
    } 
    else 
    {
        return nullptr;
    }
}

int main() {
    int goods;
    std::cout << "请输入货物位置";
    std::cin >> goods;
    float (*routeArray)[2] = getRoute(goods); // 获取对应的数组指针

    if (routeArray != nullptr) {
        // 根据选择的路由输出数组的内容
        int size = (goods == 1) ? 5 : 7;  // 根据路由选择确定数组大小
        for (int i = 0; i < size; ++i) {
            std::cout << "点 " << i + 1 << ": (" << routeArray[i][0] << ", " << routeArray[i][1] << ")\n";
        }
    } else {
        std::cout << "无效的路由选择。\n";
    }

    return 0;
}