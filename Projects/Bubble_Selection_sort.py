def bubbleSort(arr, n):
    for i in range(n-1):
        for j in range(0, n-i):
            if arr[j]>arr[j+1]:
                arr[j],arr[j+1] = arr[j+1],arr[j]

def selectionSort(arr, n):
    for j in range(n):
        large = arr[0]
        index = 0
        for i in range(n-j):
            if arr[i+1] > arr[i]:
                large = arr[i+1]
                index = i+1
        arr[n-j],arr[index] = arr[index],arr[n-j]

#Driver Code
arr = []
arr = [2,3,4,7,5,3,2]
n = len(arr)

print("The orignal array is : \n")
for i in range(n):
    print(arr[i])

selectionSort(arr, (n-1))

print("The sorted array is : \n")
for i in range(n):
    print(arr[i])
