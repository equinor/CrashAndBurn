def reverseZeroOne(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0:
                matrix[i][j] = 1
            else:
                matrix[i][j] = 0
    return matrix
