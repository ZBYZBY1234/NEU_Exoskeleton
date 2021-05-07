import numpy as np
'''平方差误差函数'''


class SquareError:
    def normal(self, h, y):
        return 0.5 * (y - h) ** 2

    def delta(self, h, y):
        return -(y - h)

'''测试差误差函数'''
class TestError:
    def normal(self, h, y):
        output = np.zeros(len(y))
        for each in h:
            output = output + each
        return output

    def delta(self, h, y):
        return np.ones((len(h[-1])))