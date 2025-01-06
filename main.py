import numpy as np


def max_unique_sequence_length(input_array: np.ndarray) -> int:
    """
    Находит максимальную длину последовательности одинаковых элементов в массиве.

    :param input_array: Входной массив.
    :return: Максимальная длина последовательности одинаковых элементов.
    """

    # Находим индексы, где происходит изменение значений в массиве.
    change_indices = np.where(np.diff(input_array))[0] + 1

    # np.hstack объединяет массивы (горизонтально).
    # np.diff вычисляет разности между соседними элементами, чтобы найти длины последовательностей.
    # np.max позволяет получить максимальную длинну такой последовательности.
    return np.max(np.diff(np.hstack((0, change_indices, input_array.size))))


def main() -> None:
    arr = np.array([22, 1, 1, 1, 1, 1, 1, 99, 2, 3, 999, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 4, 4, 5, 54, 34])
    print(max_unique_sequence_length(arr))


if __name__ == '__main__':
    main()
