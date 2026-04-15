// MUVI 检测测试文件
// 包含多个全局变量，其中一些应当总是被一起更新

#include <stdio.h>

// 模拟关联变量对: count 和 sum 总是一起被更新
int count = 0;
int sum = 0;

// 模拟关联变量对: width 和 height 总是一起被更新
int width = 0;
int height = 0;

// 独立变量
int flag = 0;

void update_stats(int val) {
    count++;      // count 和 sum 一起更新
    sum += val;
}

void update_stats2(int val) {
    count++;      // count 和 sum 一起更新
    sum += val;
}

void resize(int w, int h) {
    width = w;    // width 和 height 一起更新
    height = h;
}

void resize2(int w, int h) {
    width = w;    // width 和 height 一起更新
    height = h;
}

// BUG: 这里只更新了 count 而忘记更新 sum —— MUVI 应该能发现这个异常
void buggy_update(int val) {
    count++;
    // sum += val;  // 遗漏！
    flag = 1;
}

// BUG: 只更新了 width 而忘记更新 height
void buggy_resize(int w, int h) {
    width = w;
    // height = h;  // 遗漏！
}

int main() {
    update_stats(10);
    update_stats2(20);
    resize(100, 200);
    resize2(300, 400);
    buggy_update(30);
    buggy_resize(500, 600);

    printf("count=%d sum=%d\n", count, sum);
    printf("width=%d height=%d\n", width, height);
    return 0;
}
