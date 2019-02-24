---
title: C++基础——多态
date: 2019-02-24 15:15:23 +0800
layout: post
permalink: /blog/2019/02/24/polymorphism.html
categories:
  - 代码基础
tags:
  - C++
---

# 多态(polymorphism)

### 虚函数实现多态

C++多态(polymorphism)是通过虚函数来实现的，虚函数允许子类重新定义成员函数，而子类重新定义父类的做法称为覆盖(override)，或者称为重写。

在基类中声明指针,利用**指针**指向子类的对象,并调用对应的虚函数具体实现,进行**动态绑定**.

```c++
#include<iostream>  
using namespace std;  
  
class A  
{  
public:  
    void foo()  
    {  
        printf("1\n");  
    }  
    virtual void fun()  
    {  
        printf("2\n");  
    }  
};  
class B : public A  
{  
public:  
    void foo()  //隐藏：派生类的函数屏蔽了与其同名的基类函数
    {  
        printf("3\n");  
    }  
    void fun()  //多态、覆盖
    {  
        printf("4\n");  
    }  
};  
int main(void)  
{  
    A a;  
    B b;  
    A *p = &a;  
    p->foo();  //输出1
    p->fun();  //输出2
    p = &b;  
    p->foo();  //取决于指针类型，输出1
    p->fun();  //取决于对象类型，输出4，体现了多态
    return 0;  
} 
--------------------- 
作者：i_chaoren 
原文：https://blog.csdn.net/i_chaoren/article/details/77281785 
```

### 纯虚函数和普通虚函数

纯虚函数是在基类中声明的虚函数，它在基类中没有定义，但要求任何派生类都要定义自己的实现方法。在基类中实现纯虚函数的方法是在函数原型后加`=0` 。

#### 引入纯虚函数的原因：

* 为了方便使用多态特性，我们常常需要在基类中定义虚拟函数。 

* 在很多情况下，基类本身生成对象是不合情理的。例如，动物作为一个基类可以派生出老虎、孔雀等子类，但动物本身生成对象明显不合常理。 

虚函数是C++中用于实现多态的机制。核心理念就是通过基类访问派生类定义的函数。如果父类或者祖先类中函数func()为虚函数，则子类及后代类中，函数func()是否加virtual关键字，都将是虚函数。为了提高程序的可读性，建议**后代中虚函数都加上virtual关键字**。

### C++保留字 override

override 仅在成员函数声明之后使用时才是区分上下文的且具有特殊含义；否则，它不是保留的关键字。使用 override 有助于防止代码中出现意外的继承行为。

以下示例演示在未使用override 的情况下，可能不打算使用派生类的成员函数行为。编译器不会发出此代码的任何错误。

### without override

```C++
class BaseClass
{
  virtual void funcA();
  virtual void funcB() const;
  virtual void funcC(int = 0);
  void funcD();
};
 
class DerivedClass: public BaseClass
{
  virtual void funcA(); // ok, works as intended
 
  virtual void funcB(); // DerivedClass::funcB() is non-const, so it does not
             // override BaseClass::funcB() const and it is a new member function
 
  virtual void funcC(double = 0.0); // DerivedClass::funcC(double) has a different
                   // parameter type than BaseClass::funcC(int), so
                   // DerivedClass::funcC(double) is a new member function
};
```

### with override will throw error

```C++
class BaseClass
{
  virtual void funcA();
  virtual void funcB() const;
  virtual void funcC(int = 0);
  void funcD();
};
 
class DerivedClass: public BaseClass
{
  virtual void funcA() override; // ok
 
  virtual void funcB() override; // compiler error: DerivedClass::funcB() does not 
                  // override BaseClass::funcB() const
 
  virtual void funcC( double = 0.0 ) override; // compiler error: 
                         // DerivedClass::funcC(double) does not 
                         // override BaseClass::funcC(int)
 
  void funcD() override; // compiler error: DerivedClass::funcD() does not 
              // override the non-virtual BaseClass::funcD()
};
```

**公有继承包含两部分：一是“接口”(interface)，二是 "实现" (implementation)。**

```C++
class Person{
public:
    virtual void Eat() const = 0;    // 1) 纯虚函数
    virtual void Say(const std::string& msg);  // 2) 普通虚函数
    int Name() const;  // 3) 非虚函数
};
 
class Student: public Person{ ... };
class Teahcer: public Person{ ... };
--------------------- 
原文：https://blog.csdn.net/fanyun_01/article/details/79122136 
```

* 纯虚函数: 继承的是基类成员函数的接口，必须在派生类中重写该函数的实现,若想调用基类的 Eat()，须加上 类作用域操作符 ``::``

* 普通虚函数: 普通虚函数，对应在基类中定义一个缺省的实现 (default implementation)，表示继承的是基类成员函数的**接口和缺省的实现**，由派生类自行选择是否重写该函数。实际上，允许普通虚函数**同时继承接口和缺省实现是危险的**。(如果不小心在派生类中忘记重写,则调用父类而不是子类中缺省的)

  **所以要么用 纯虚函数+缺省实现, 因为父类的缺省实现必须显示调用, 所以忘了也没事**

  **要么用C++11中关键字override,避免一不小心**

非虚成员函数没有virtual关键字，表示派生类不但继承了接口，而且继承了一个强制实现（mandatory implementation），既然继承了一个强制的实现，此时，派生类中重新定义的成员函数会 “隐藏” (hide) 继承自基类的成员函数, 这是因为非虚函数是 “静态绑定” 的，p被声明的是 Person* 类型的指针，则通过 p调用的非虚函数都是基类中的，即使 p指向的是派生类。  

#### override 好处都有啥

在派生类中，重写 (override) 继承自基类成员函数的实现 (implementation) 时，要满足如下条件：

* 一虚：基类中，成员函数声明为虚拟的 (virtual)
* 二容：基类和派生类中，成员函数的返回类型和异常规格 (exception specification) 必须兼容
* 四同：基类和派生类中，成员函数名、形参类型、常量属性 (constness) 和 引用限定符 (reference qualifier) 必须完全相同

如此多的限制条件，导致了虚函数重写如上述代码，极容易因为一个不小心而出错.
  C++11 中的 override 关键字，可以显式的在派生类中声明，哪些成员函数需要被重写，如果没被重写，则编译器会报错。

### 总结

1)  公有继承
　　纯虚函数      => 继承的是：接口 (interface)
　　普通虚函数   => 继承的是：接口 + 缺省实现 (default implementation)
　　非虚成员函数 =>继承的是：接口 + 强制实现 (mandatory implementation)
2)  不要重新定义一个继承自基类的非虚函数 (never redefine an inherited non-virtual function
3)  在声明需要重写的函数后，加关键字 override
这样，即使不小心漏写了虚函数重写的某个苛刻条件，也可以通过编译器的报错，快速改正错误。

---------------------
作者：老樊Lu码 
原文：https://blog.csdn.net/fanyun_01/article/details/79122136 