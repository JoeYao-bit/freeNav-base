//
// Created by yaozhuo on 2022/5/14.
//

#include "dependencies/self_sorted_queue.h"
#include "gtest/gtest.h"
using namespace freeNav;

TEST(SORTED_QUEUE, SIMPLE_TEST) {

    AutoSortQueue<double, double> queue;

    std::vector<std::pair<double, double> > pair_values = {{1, 1}, {3,3}, {2.2, 2.2}};

    for(const auto& pair_val : pair_values) {
        auto v1 = std::make_shared<NodeId>(MAX<int>);
        SortPair<double, double> sp = std::make_pair(pair_val.first, withIndex<double>(pair_val.second, v1));
        queue.insertMin(sp);
    }

    std::cout << "sorted queue = " << AutoSortQueue<double, double>::toValueString(queue.queue()) << std::endl;

}

//TEST(SortHeap, SimpleTEst) {
//    SortQueue<double, double> heap(10, {DBL_MAX, DBL_MAX});
//    HeapInsert(heap, {.1, .1});
//    HeapInsert(heap, {.1, .1});
//    HeapInsert(heap, {.3, .3});
//    HeapInsert(heap, {1., 1.});
//    HeapInsert(heap, {.6, .6});
//    HeapInsert(heap, {-1.1, -1.1});
//    HeapInsert(heap, {-3.1, -3.1});
//
//    std::cout << "sorted queue = " << AutoSortQueue<double, double>::toValueString(heap) << std::endl;
//
//}

#define TOTAL 20

TEST(RAW_HEAP, SIMPLE_TEST)
{
    int heap[TOTAL];
    int num;
    int i;

    //先输入一半的数据，对输入的数组建堆。
    printf("输入Total/2个数据：\n");
    for (i = 0; i < TOTAL / 2; i++) {
        //scanf("%d", &heap[i]);
        heap[i] = 4 - i;
    }

    CreatHeap(heap, TOTAL / 2);

    //检验是否建堆成功。
    printf("建堆后：\n");
    for (i = 0; i < TOTAL / 2; i++)
        printf("%-3d", heap[i]);
    putchar('\n');

    //向已建好的堆中插入数据，并重组为堆。
    printf("继续输入Total/4个数据：\n");
    for (i = TOTAL / 2; i < TOTAL / 2 + TOTAL / 4; i++)
    {
        //scanf("%d", &num);
        num = pow(1.1, i);
        HeapInsert(heap, i, num);
    }

    //检验是否插入成功。
    printf("重组为堆之后：\n");
    for (i = 0; i < TOTAL / 2 + TOTAL / 4; i++)
        printf("%-3d", heap[i]);
    putchar('\n');

    //删除堆顶元素Total/4次。
    printf("删除Total/4个数据：\n");
    for (i = 0; i < TOTAL / 4; i++)
        HeapDelete(heap, TOTAL / 2 + TOTAL / 4 - i);

    //检验是否删除成功。
    for (i = 0; i < TOTAL / 2; i++)
        printf("%-3d", heap[i]);
    putchar('\n');

    //向堆中插满数据，进行堆排序。
    printf("继续输入Total/2个数据：\n");
    for (i = TOTAL / 2; i < TOTAL; i++)
    {
        //scanf("%d", &num);
        num = pow(-2, i);
        HeapInsert(heap, i, num);
    }

    HeapSort(heap, TOTAL);
    printf("排序后：\n");
    for (i = 0; i < TOTAL; i++)
        printf("%-3d ", heap[i]);
    putchar('\n');
}


TEST(AutoSortHeap, SimpleTest) {
    AutoSortHeap<double, double> ash(100);

    std::vector<std::pair<double, double> > pair_values = {
            {4.2, 4.2}, {1.1, 1.1},
            {1.3,1.3}, {2.2, 2.2},
            {-2.5, -2.2}, {2.2, 2.2},
            {-2.4, -2.2}, {2.2, 2.2},
            {-2.22, -2.2}, {2.2, 2.2},
            {-12.2, -12.2}, {12.2, 12.2},
            {-24.2, -2.2}, {24.2, 2.2},
            {-32.2, -12.2}, {32.2, 12.2},
            };

    std::vector<std::shared_ptr<NodeId> > int_ptrs;

    for(const auto& pair_val : pair_values) {
        //std::cout << " flag 1" << std::endl;
        auto v1 = std::make_shared<NodeId>(MAX<NodeId>);
        int_ptrs.push_back(v1);
        //std::cout << " flag 2" << std::endl;
        SortPair<double, double> sp = std::make_pair(pair_val.first, withIndex<double>(pair_val.second, v1));
        //std::cout << " flag 3" << std::endl;
        ash.HeapInsert(sp);
    }

    std::cout << "heap " << ash.toKeysString() << std::endl;

    std::cout << "delete " << ash.HeapPopMin().first << std::endl;
    std::cout << "delete " << ash.HeapPopMin().first << std::endl;
    std::cout << "delete " << ash.HeapPopMin().first << std::endl;

    std::cout << "heap " << ash.toKeysString() << std::endl;

    ash.updateValue(3, -10);
    std::cout << "after update heap " << ash.toKeysString() << std::endl;
    std::cout << "delete " << ash.HeapPopMin().first << std::endl;

    ash.updateValue(2, -11);
    std::cout << "after update heap " << ash.toKeysString() << std::endl;
    std::cout << "delete " << ash.HeapPopMin().first << std::endl;

    std::cout << "heap " << ash.toKeysString() << std::endl;

}