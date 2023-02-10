// container: C++ function

/*************************************************/
#include<unistd.h>
usleep(100); // unit is us(micro seconds)
/*************************************************/
#include <cmath>
// returns the smallest possible integer value which is greater than or equal to the given argument. 不小于给定值的最小整数
cout << ceil(15.08); // outut 16
// 不大于给定值的最大整数
cout << floor(15.08); // outut 15
/*************************************************/
// std::fill函数的作用是：将一个区间的元素都赋予指定的值，即在[first, last)范围内填充指定值。
// std::fill_n函数的作用是：在[fist, fist + count)范围内填充指定值。
std::move //???
std::vector<int> myvector(8); // myvector: 0 0 0 0 0 0 0 0
std::fill(myvector.begin(), myvector.begin() + 4, 5); // myvector: 5 5 5 5 0 0 0 0
std::fill(myvector.begin() + 3, myvector.end() - 2, 8); // myvector: 5 5 5 8 8 8 0 0
std::vector<int> p(myvector.begin(), myvector.end()); //用首尾指针初始化向量
/*************************************************/
// execute statements in try
// 如果执行的过程中没有异常拋出，skip all catch
// 如果拋出了异常，立即跳转到第一个“异常类型”和拋出的异常类型匹配的 catch 块中执行（称作异常被该 catch 块“捕获”），执行完后skip all catch
// 如果一个函数在执行过程中拋出的异常在本函数内就被 catch 块捕获并处理，那么该异常就不会拋给这个函数的调用者（也称为“上一层的函数”）；如果异常在本函数中没有被处理，则它就会被拋给上一层的函数。
try {if( n == 0) throw -1;} //抛出int类型异常
catch(double d) { cout << "catch(double) " << d <<  endl;}
catch(int e) { cout << "catch(int) " << e << endl;}
catch (string s ) {throw;} //继续抛出捕获的异常给上一层的函数
catch (...) { cout << "catch (...)" << endl;} // 这样的 catch 块能够捕获任何还没有被捕获的异常
/*************************************************/
dynamic_cast < new-type> ( expression ); // 该运算符把expression转换成new-type类型的对象。
/*************************************************/
#include <map> // STL头文件没有扩展名.h
// https://www.w3cschool.cn/cpp/cpp-fu8l2ppt.html
std::map<int , std::string> mapPerson;

mapPerson.insert(pair < int,string > (1,"Jim"));// insert 函数插入 pair 数据
mapPerson.insert(std::map < int, std::string > ::value_type (2, "Tom"));// insert 函数插入 value_type 数据
mapPerson[3] = "Jerry";// 用数组方式插入数据
// 前向迭代器
std::map < int ,std::string > ::iterator it;
std::map < int ,std::string > ::iterator itEnd;
it = mapPerson.begin();
itEnd = mapPerson.end();
// 反向迭代器
std::map < int, string > ::reverse_iterator iter;
it = mapPerson.rbegin();iter = mapPerson.rend();
//查找
it = maplive.find(112);// find() 函数返回一个迭代器指向键值为 key 的元素，如果没找到就返回指向 map 尾部的迭代器。
it = mapPerson.upper_bound();// 返回键值=给定元素的第一个位置;
it = maplive.lower_bound();// 返回键值<给定元素的第一个位置
//删除
iterator erase(iterator it);//通过一个条目对象删除
iterator erase(iterator first，iterator last);//删除一个范围
size_type erase(const Key&key);//通过关键字删除	
/*************************************************/
std::lock_guard<std::mutex> lock(mtx); // 离开局部作用域，析构函数自动完成解锁功能
/*************************************************/
//参数首先传入括号中的形参，然后形参传入基类的构造函数people并完成另一个变量的赋值
Student::Student(char *name, int age, float score): People(name, age), m_score(score) { }
/*************************************************/
//for auto遍历
std::deque<State> imuIntegrationQueue;
for(auto &it : imuIntegrationQueue) {} // 可以修改队列中的元素;在循环体中，it的数据类型为State
for(auto it  : imuIntegrationQueue) {} // 不能修改队列中的元素
/*************************************************/
