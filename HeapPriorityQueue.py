class PriorityQueueBase:
    class Item:
        __slots__ = '_key', '_value'

        def __init__(self, k, v):
            self._key = k
            self._value = v

        def __lt__(self, other):
            return self._key < other._key


class HeapPriorityQueue(PriorityQueueBase):

    def _parent(self, j):
        return (j - 1) // 2

    def _left(self, j):
        return 2 * j + 1

    def _right(self, j):
        return 2 * j + 2

    def _has_left(self, j):
        return self._left(j) < len(self._data)

    def _has_right(self, j):
        return self._right(j) < len(self._data)

    def _swap(self, i, j):
        self._data[i], self._data[j] = self._data[j], self._data[i]

    def _upheap(self, j):
        parent = self._parent(j)
        if j > 0 and self._data[j] < self._data[parent]:
            self._swap(j, parent)
            self._upheap(parent)

    def _downheap(self, j):
        small_child = 0
        if self._has_left(j):
            left = self._left(j)
            small_child = left

            if self._has_right(j):
                right = self._right(j)
                if self._data[right] < self._data[left]:
                    small_child = right

            if self._data[small_child] < self._data[j]:
                self._swap(j, small_child)
                self._downheap(small_child)

    def __init__(self):
        self._data = []

    def __len__(self):
        return len(self._data)

    def is_empty(self):
        return len(self._data) == 0

    def add(self, key, value):
        self._data.append(self.Item(key, value))
        self._upheap(len(self._data) - 1)

    def min(self):

        if self.is_empty():
            raise Exception('Priority queue is empty.')
        item = self._data[0]
        return (item._key)

    def min1(self):

        if self.is_empty():
            raise Exception('Priority queue is empty.')
        item = self._data[0]
        return (item._value)

    def min_value(self):
        if self.is_empty():
            raise Exception('Priority queue is empty.')
        item = self._data[0]
        return (item._value)

    def remove_min(self):
        if self.is_empty():
            raise Exception("Priority queue is empty.")
        self._swap(0, len(self._data) - 1)
        item = self._data.pop()
        for i in range(0, len(self._data)):
            if (i + 1) < (len(self._data)) and self._data[i]._key == self._data[i + 1]._key:
                self._swap(i, i + 1)
            else:
                break
        self._downheap(0)
        return (item._value)

    def print_Queue(self):
        for i in range(0, len(self._data)):
            x = self._data[i]
            print(x._key, x._value)

    def search_Queue(self, key, value):
        for i in range(0, len(self._data)):
            x = self._data[i]
            if x._value == value:
                return i, x._value
        return (-1, (-1, -1))

    def remove_index(self, k):
        d = self._data[len(self._data) - 1]
        self._swap(k, len(self._data) - 1)
        if k == len(self._data) - 1:
            self._data.pop()
            return
        else:
            item = self._data.pop()
            if d._key <= item._key:
                self._upheap(k)
            else:
                self._downheap(k)

    def my_search(self, value):
        for i in range(0, len(self._data)):
            x = self._data[i]
            if x._value == value:
                return x._key, x._value
        return (-1, (-1, -1))

    def remove_g_index(self, k):
        d = self._data[len(self._data) - 1]
        self._swap(k, len(self._data) - 1)
        if k == len(self._data) - 1:
            item = self._data.pop()
        else:
            item = self._data.pop()
            if d._key <= item._key:
                self._upheap(k)
            else:
                self._downheap(k)
        return (item._value)

    def find_f(self, k):
        for i in range(0, len(self._data)):
            k = self._data[i]
            if k._key == k:
                return i, k._key

        return (-1, (-1, -1))

    def tolist(self, val):
        l = []
        for i in range(0, len(self._data)):
            if val == self._data[i]._key:
                l.append((self._data[i]._key, self._data[i]._value))
        return l

    def remove_all(self):
        del self._data[:]
