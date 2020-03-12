def partition(arr, low, high):
    i = low-1
    pivot = arr[high]

    for j in range(low,high):
        if arr[j] <= pivot:

            i = i + 1
            arr[i],arr[j] = arr[j],arr[i]

    arr[i+1],arr[high] = arr[high],arr[i+1]
    return (i+1)

def quickSort(arr, low, high):

    if low < high:
        pi = partition(arr,low,high)

        quickSort(arr, low, pi-1)
        quickSort(arr, pi+1, high)

#DRIVER code
arr = [1,55,32,56,31,22,69,78,25,55]
n = len(arr)

print("orignal Array is : \n")
for i in range(n):
    print(arr[i],"\t")

quickSort(arr,0,(n-1))

print("Sorted Array is : \n")
for i in range(n):
    print(arr[i],"\t")
