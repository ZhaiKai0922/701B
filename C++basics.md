# C++ 基础

### 函数：

​		一般能被反复调用的代码，可以接收输入，进行处理并产生输出

- 返回类型：表示了函数返回结果的类型
- 函数名：用于函数调用
- 形参列表：表示函数接收的参数类型，可以为空，可以为void，可以无形餐
- 函数体：具体的执行逻辑

main函数：特殊的函数，作为整个程序的入口

- 返回类型为int，表示程序的返回值，通常使用0表示正常返回。
- 形参列表可以为空

```cpp
#include <iostream>

//返回类型 函数名 形参列表
int main()   //（）包含形参
int main(int argc, char* argv[]) //
//函数体
{
    std::cout << "Hello world!"<<std::endl;
    
    return 0;   //通常返回0表示成功
}
```



## C++ 初探

```cpp
#include <iostream> //系统会在系统库中寻找头文件，C++标准库没有后缀
#include "a.hpp"  //系统将在当前文件的目录中寻找头文件，当前project

//是否立即刷新缓冲区
//std::cout std::cerr std::clog
//缓冲器刷新机制：std::flush std::endl
//如果想换行，但是不想刷新缓冲区，可以采用字符\n 如:"output from cout\n"
```

  ### 名字空间：

```cpp
namespace NameSpace1
{
    void fun()
    { 
    }
}
namespace NameSpace2
{
    void fun()
    {    
    }
}

int main()
{
    
}
```

```cpp
#include <cstdio>
//c的IO
printf("Hello Word\n");

int x = 10;  //任何一个数据类型，都保存在内存之中。
std::cout << "I"<<x<<"you\n";   //可以匹配x的类型。
printf("I %d you\n", x);    //%d表示int型，替换的符号要一致。

```



### 猜数字与控制流

if语句：

```cpp
#include <iostream>
#include <cstdio>

int main()
{
    int x = 42;
    std::cout << "Please input a number:";

    int y = 0;
    std::cin >> y;

    if(y == x)
    {
        std::cout << "You are right!\n";
    }
    else
    {
        std::cout << "You are Wrong!\n";
    }
}
```

用于分支选择的if语句

- 条件部分：用于判断是否执行，返回bool值，如果是真就执行if语句部分。
- 语句部分：要执行的操作

==与=操作

=是赋值操作，`int y = 0`

==是比较，返回`bool`值

x = y = 42;

赋值表达式，从右向左执行，**赋值表达式的返回值是42**。



### 结构体与自定义数据类型

结构体：将相关的数据放置在一起

- 可以通过（.）操作符访问内部元素

- 可以作为函数的输入参数和返回类型
- 可以引入成员函数，更好地表示函数与数据的相关性

```cpp
#include <iostream>

//结构体
struct Point
{
    int x;
    int y;
};

void fun(Point& p)
{
    p.x = p.x + 1;
    p.y = p.y + 1;
}

int main()
{
    Point p;      //p的类型为用户自定义类型Point
    p.x = 10;
    p.y = 12;
    fun(p);
    std::cout << p.x<<"\n"<<p.y<<"\n";
}
```





## 第2章 对象与基本类型

### 从初始化/赋值语句谈起

值与对象都有类型。

初始化与赋值可能涉及到类型转换。 

C++是强类型语言

引入类型是为了更好地描述程序，防止误用。

类型描述了：

- 存储所需要的尺寸（`sizeof`，标准并没有严格限制, 1 byte -> 8 bit）
- 取值空间(std::numeric)  int取值最小值-2147483648 最大值 2147483648 unsigned int 取值最小值为0，最大值为4294967295，最大值+1 变成最小值；最小值-1变成最大值。

- 对齐信息，对结构体的大小会产生影响
- 可以执行的操作

```cpp
#include <iostream>
#include <limits>
int main()
{
    char t; //char一般为1个字节，8位，256种
    char16_t t2;    //可以表示一些复杂的符号 2个字节 16位
    char32_t t3;    //4个字节 32位

    //int i;  //4个字节 32位 正负都可以取值
    //unsigned int j;  //4个字节 32位 去大于等于0

    float i = -100.020;

    std::cout << i<<std::endl;

    std::cout << std::numeric_limits<float>::min()<<std::endl;
    std::cout << std::numeric_limits<float>::max()<<std::endl;
    std::cout << std::numeric_limits<double>::min()<<std::endl;
    std::cout << std::numeric_limits<double>::max()<<std::endl;
}
```



### 类型详述

类型可以划分为基本类型与复杂类型

基本类型：C++语言中所支持的类型

数值类型

- 字符类型（char, wchar_t, char16_t, char32_t）

- 整数类型

  带符号的整数类型：short, int, long, long long

  无符号的整数类型：unsigned+带符号的整数类型

- 浮点类型

  float, double, long double float浮点类型 double 双精度

  float占4字节，即32位

  double占8字节，即64位

  long double占16字节，即128位

void 是一个特殊的类型

​		复杂类型：由**基本类型**组合、变种所产生的类型，可能是**标准库**引入，或者**自定义类型**。

与类型相关的标准未定义部分：

- char是否有符号

- 整数中内存中的保存方式：大端法，小端法；

- 每种类型的大小(间接影响取值范围)

  C++11中引入了**固定尺寸**的整数类型，如int32_t;

```cpp
unsigned char ch1;
signed char ch2;
```



### 字面值及其类型

- 整数字面值：20（十进制），024（八进制），0x14（十六进制） --int型
- 浮点型：1.3，1e8（e8表示*10的8次方） --double型
- 字符字面值：'c', '\n' --char型
- 字符串字面值："Hello" -char[6]型  "Hello\0"  \0是最后一个字符
- bool型 true false
- 指针字面值：nullptr - nullptr_t型

```cpp
float i = 1.3f;
//可以引入自定义后缀来修改字面值类型
```



### 变量及其类型

变量：对应了一段存储空间，可以改变其中内容

变量的类型在其首次声明（定义）时指定：

- int x:定义一个变量x，其类型为int
- 变量声明与定义的区别：**extern前缀**

变量的初始化与赋值

- 初始化：在构造变量之初为其赋予的初始值

  缺省初始化：int x;  没有显示的赋予初始值，根据变量声明的位置，在全局的变量为0，在函数内部为随机的值。

  直接/拷贝初始化：

  其他初始化

- 赋值：修改变量所保存的数值

为变量赋值时可能涉及到类型的转换

- `float x = 1.3`涉及到double到float的类型转换。

类型转换不止发生在赋值：

- if(3)

- 数值比较时 会发生隐式的数据类型转换

  ```cpp
  #include <iostream>
  int x = -1;
  unsigned int y = 3;
  std::cout << (x < y)<<std::endl; //会把int型转换为unsigned int
  ```



### 复合类型：从指针到引用

**指针**：一种间接类型

指针也开辟一块内存，其中内存里面存放的是地址，其中内存空间本身也有编号。

特点： 可以“指向”不同的对象，具有相同的尺寸

相关操作：&取地址（变量地址）操作符 *解引用操作符			



指针的定义

```cpp
int* p = &x;   //为指针进行初始化，这是一个拷贝初始化。
//缺省初始化，虽然效率高，但是不稳定
int* p = nullptr;  //nullptr是一个特殊的对象，表示空指针。
int* p = NULL;  //NULL就是0，但是更加安全。
int* p = 0;  //0是字面值，int型，int型的字面值赋给int*型 
```

指针与bool的隐式转换：

非空指针可以转变为true；

空指针转变为false；

指针的+1与-1是+或者-整个int型的大小，而不是就一个单独的字节

```cpp
#include <iostream>

int main()
{
    int x = 1;
    char y = 'r';

    int* p = &x;                                          //&取址符

    std::cout << p<<"\n";                     //打印p：0x7fffffffdb3c

    p = p + 1;                                              //p = p + 1

    std::cout << p <<"\n";                     //打印p：0x7fffffffdb40

    p = p - 1;
    
    std::cout << p<<"\n";                    //打印p：0x7fffffffdb3c
}
```



指针的判断，返回0与1：

```cpp
std::cout << (p == q) << "\n"；
```

void* 指针

void* 指针可以转换为任意的指针，任意的指针都可以转换为 void*

void* 指针不能进行+1-1操作，因为不知道具体字节大小

指针的指针

自定义类型比较大，复制成本比较高，所以要采用指针类型，提升效率。

函数传入指针，指针就是8个字节，只是一个地址，不会占用很大空间。



函数传值，只是一个拷贝。

函数传指针（传址），可以改变相应的传入参数的值。



#### 引用：

引用是对象的别名；

```cpp
int x = 3;
int& ref = x;

int& ref = 3;  //错误，引用是对象的别名，不能绑定字面值。

int* ptr = &x;

//构造时，引用就要绑定到一个具体的对象，不能缺省初始化。
```



### 常量类型与常量表达式

#### 常量指针

```cpp
int* const ptr = &x;    //表示指针不能修改，只能指向x

const int* ptr = &x;   //表示不能改变ptr指针指向的值
```



常量引用

```cpp
int x = 3;
const int& ref = x;
```

**函数的形参**



### 类型别名与类型的自动推导

```cpp
typedef int MyInt;   //类型别名 
using MyInt = int;    //(从C++11开始)
using MyCharArr = char[4];
```



### 域与对象的生命周期

全局域：

块域：

**域可以嵌套，嵌套域中定义的名称可以隐藏外部中定义的名称。**

对象的生命周期：

```cpp
#include <iostream>

int x = 3;    //全局对象的生命周期是整个程序的运行期间。
//局部对象生命周期起源于对象的初始化位置，终止于所在域被执行完成。
```

**全局对象的生命周期是整个程序的运行期间**

**局部对象的生命周期起源于对象的初始化位置，终止于所在域被执行完成**





## 数组、vector与字符串

### 1. 数组

- 将一到多个相同类型的对象串到一起，所组成的类型

  int a -> int b[10]

```cpp
int main()
{
    int a;          //int类型
    int b[10];  //int[10]类型  类型这个信息是编译器处理的，所以类型必须提前确认，int b[x]就不可以！！！
}
```

- 数组初始化

  ```cpp
  int b[3] = {1, 2}; //前两个元素初始化，后面元素用0初始化
  int b[3] = {};  //全部用0初始化
  
  int b[] = {1, 2, 3};  //简化写法，编译器直接推导出int[3]；
  //数组的大小必须确定
  ```

  注意事项：

  不能使用auto。

  数组不能进行复制。

数组的复杂声明：

- 指针数组

  ```cpp
  int* a[3];  //数组a中包含了3个元素，都是int*型
  ```

- 数组的指针，数组的引用(不能定义引用的数组)

  ```cpp
  int (*a)[3]; 
  
  int b[3]; 
  int (&a)[3] = b;
  ```

```cpp
int a[3] = {1, 2, 3};

std::cout << a << &(a[0]) << std::begin(a) << std::endl;
std::cout << a + 3 << &(a[3]) << std::end(a) << std::endl;   //其中末尾指针指向的是最后一个元素+1 的位置
```



```cpp
//数组的其他操作
//数组的求个数
//sizeof()求元素个数。
sizeof(a)/sizeof(int);
std::size(a)     //推荐方法 
```



```cpp
for(int x : a)
{
    std::cout <<  x <<std::endl;
}
```



#### 多维数组

```cpp
int x[][] = {1, 2, 3, 4};  //这是不能通过编译的

int x[][3] ;  //不建议这样初始化，太混乱

x[0][3];  //遍历

//遍历多维数组，采用多重循环
for(auto& p : x2)
{
    for(auto q : p)
        std::cout << q << std::endl;
}

// 
```



#### vector

是一个标准库中定义的一个类模板

与内建数组相比，更侧重于易用性

```cpp
#include <iostream>
#include <cstring>
#include <vector>

int main()
{
    std::vector<int> x;  //std名字空间，缺省初始化，包含0个元素
    std::vector<float> y;  //数组不支持复制，但是vector支持复制
    //vector可以在运行期间 改变元素个数

    std::vector<int> z = {1, 2, 3};  //聚合方式初始化
    std::vector<int> h(10);   //元素都为0
    std::vector<int> g(3, 1);   //3个元素都为1

    //其他方法
    std::cout << x.size() << std::endl;   //std::size(a) 
    std::cout << x.empty() << std::endl;   //std::size(a)

    z.push_back(2);

    std::cout << z.front()<< "\n";

    auto b = z.begin();     //iterator 迭代器，模拟指针的行为，可以解引用，可以下标访问 ，可以移动。
    while(b != z.end())
    {
        std::cout << *b << std::endl;
        b = b + 1;
    }

    b = z.begin();
    std::cout << b[1] <<std::endl;  //2
    for(auto val : z)
    {
        std::cout << val << std::endl;
    }

    z.pop_back();   //从末尾弹出元素 

     //可以进行vector的比较
    std::cout << (x == z) << std::endl;
    std::cout << (x > z) << std::endl;
    std::cout << (x < z) << std::endl;

     //元素访问
     x[2];   //与数组一致
     x.at(2);   //与上面一致，但是越界会返回

     size_t h1[3] = {1, 2, 3};
     size_t* h2 = std::begin(h1);   //获取指向第一个元素的指针
     size_t* h3 = std::end(h1); //获取指向最后一个元素 下一个的指针
     std::cout << h3 - h2 << std::endl;

     std::vector<std::vector<int>> vec;
     vec.push_back(std::vector<int>());
     vec[0].push_back(1);
     return 0;
}
```



#### string

C++类模板

```cpp
#include <iostream>
#include <cstring>
#include <vector>
#include <string>

int main()
{
    char* c = "Hello";
    std::cout << c[4]<<std::endl;

    std::string str = "Hello";
    str = str + "World";
    std::cout << str<<"\n";

    std::string str2(3, 'a'); 
    std::cout << str2<<std::endl;

    std::cout << (str == str2)<<"\n";

    str = "New Hello";

    auto ptr = str.c_str();    //转换为c风格字符串，const char* ptr

    const char* ptr2 = str.c_str();

    std::cout << ptr2 << std::endl;
}
```





## C++表达式基础

表达式由一到多个操作数组成，可以求值并通常返回求值结果

























#### 字符串类型

```cpp
//C风格字符串
char x[] = "Hello World";

#include <string>
//C++风格字符串
string str = "Hello World";
std::cout << str;
```



#### 运算符

```cpp
//前置运算符  a = 2 ; b = ++a;  -> a = 3, b = 3;  a先递增然后赋值
//后置运算符  a = 2 ; b = a++;  -> a = 3, b = 2;  a先递增然后赋值
```



#### 程序流程结构

顺序结构，选择结构，循环结构

**选择结构：**

if(条件){条件满足执行的语句}else{条件不满足执行的语句}

多条件的if语句：if(条件1){条件1满足的语句}else if(条件2){条件2满足执行的语句} ... else{都不满足执行的语句}

**循环结构：**

while(循环条件){循环语句}

for(起始表达式；条件表达式；末尾循环体)

#### 跳转语句

**break语句：**

作用：用于跳出选择结构或者循环结构

break使用的时机

- 出现在switch条件语句中，作用是终止case并跳出switch。
- 出现在循环结构的语句中，作用是跳出当前的循环语句。
- 出现在嵌套循环中，跳出最近的内层循环语句。

```cpp
switch(select)
{
    case 1:
        ...;
        break;
    case 2:
        ...;
        break;
    default:
        break;
}
```

**continue语句：**

作用：在**循环语句**中，跳过本次循环中余下尚未执行的语句，继续执行下一次循环。



#### 二维数组定义方式

```cpp
换为c风格字符串，const char* ptr
int a[行数][列数] = {}
```



#### 函数的声明

提前告知函数存在。

```cpp
int max(int a, int b);  //声明可以有多次
```



#### 函数的分文件编写

1. 创建.h的头文件
2. 创建.cpp的源文件
3. 在头文件添加声明
4. 在源文件添加定义



#### 结构体

创建用户自定义的数据类型

语法：struct 结构体名 {结构体成员列表};

```cpp
#include <iostream>
#include <cstring>
#include <vector>
#include <string>

int main()
{
    //struct关键字 类型名称 {成员列表}
    struct student
    {
        //成员列表
        std::string name;

        int age;

        int score;
    }s3;  //顺便创建一个结构体变量

    //创建变量
    //struct关键字可以省略
    struct student s1;
    s1.name = "zhang san";
    s1.age = 18;
    s1.score = 100;

    struct student s2 = {"li si", 19, 120};

}
```



#### 结构体数组

```cpp
student s[3];		//结构体数组
student stu[3] = 
{
    {   }, {   }, {   } 
}; 
```

#### 结构体中的const的修饰场景



#### 程序内存模型——内存四区：

- 代码区：存放函数体的二进制代码，由操作系统进行管理
- 全局区：存放**全局变量**和**静态变量**以及常量
- 栈区：由编译器进行自动分配释放，存放函数的参数值，局部变量等
- 堆区：由程序员分配和释放，若程序员不释放，程序结束时由操作系统回收

不同的区域存放的数据，赋予不同的生命周期，给我们更大的灵活编程

在程序编译后，生成了.exe程序，未执行该程序之前分为两个区域

代码区：

存放**CPU执行的机器指令**

代码区是**共享**的，共享的目的是对于频繁被执行的程序，只需要在内存中有一份代码即可

代码区是**只读**的，使其只读的原因是防止程序意外地修改了它的指令

全局区：

全局变量和静态变量存放在此

全局区还包含了常量区，字符串常量和其他常量也存放在此

该区域的数据在程序结束后由操作系统释放



**函数体内的变量**都是局部变量

**没有在函数体内的变量**都是全局变量

静态变量：

```cpp
static int s_a = 10;    //静态变量

//常量
//字符串常量  全局区
//const修饰的变量
//const修饰的全局变量  全局区
//const修饰的局部变量 不存放在全局区
```

**简单总结：**存放在全局区的有 全局变量，全局常量，静态变量，字符串常量

栈区：局部变量（不包括static修饰的静态变量，字符串常量  ）



程序运行后划分的区域

栈区：

由编译器自动分配内存，存放函数的参数值，局部变量等

**注意事项：不要返回局部变量的地址，栈区开辟的数据由编译器自动释放**

```cpp
//栈区数据的注意事项  -- 不要返回局部变量的地址
//栈区数据由编译器管理开辟和释放
```

堆区：

由程序员来分配释放，若程序员不释放，程序结束时由操作系统回收

在C++中主要利用**new在堆区开辟内存**。

```cpp
int* fun()
{
    //利用new关键字开辟到堆区
    int* p = new int(10);
    return p;
}

int main()
{
    int* p = fun();

    std::cout << *p << std::endl;
    std::cout << *p << std::endl;
    std::cout << *p << std::endl;

    return 0;
}
```



#### new关键字

堆区开辟的数据，由程序员手动开辟内存，手动释放内存，释放利用操作符delete

利用New操作符创建数据，会返回该数据对象的类型的指针

```cpp
//new的基本语法
//在堆区利用new开辟空间
    
    int* fun()
{
    //利用new关键字开辟到堆区
    int* p = new int(10);
    return p;
}

void test01()
{
    int* p = fun();
    std::cout << *p <<std::endl;
}
int main()
{
    test01();
    
    return 0;
}
```

```cpp
//在堆上创建数组
void test02()
{
    int* arr = new int[10];

    for(int i = 0; i < 10; i++)
    {
        arr[i] = i + 100;
    }
    
    //释放数组
    delete []arr;
}
```



#### 引用做函数返回值 `P90`

```cpp
//引用做函数的返回值
//1. 不要返回局部变量的引用
```



函数：

函数体中占位参数，必须要传入对应的实参。

函数中的参数可以进行初始化，但是定义与声明只能初始化一个。



**函数重载**

- 同一个作用域下
- 函数名相同
- 函数的形参类型不同 或者 个数不同 或者 顺序不同

注意：函数的返回值不可以作为函数重载的条件

```cpp
//函数重载，提高函数的复用性
//必须在同一个作用域下
//函数名称相同
//函数的参数类型不同，或者个数不同，或者顺序不同
//函数返回值不能作为重载的条件
void fun()
{

}

void fun(int a)
{
    std::cout << a<<std::endl;
}

int main()
{
    fun();

    return 0;
}
```


