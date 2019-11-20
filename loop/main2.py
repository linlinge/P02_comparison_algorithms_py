from multiprocessing import Process

def f(name):
    print('hello', name)

if __name__ == '__main__':
    p = Process(target=f, args=('bob',)) # p进程执行f函数，参数为'bob'，注意后面的“,”
    p.start() # 进程开始
    p.join() # 阻塞主线程，直至p进程执行结束