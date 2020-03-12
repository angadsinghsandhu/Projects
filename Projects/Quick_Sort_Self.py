def partition(arr, low , high):
    i = low-1

    for j in range(low, high):
        if arr[j]<=arr[high]:
            arr[i],arr[j] = arr[j],arr[i]

        arr[i+1],arr[high] = arr[high],arr[i+1]

    return (i+1)

def quickSort(arr, low , high):
    if low < high:
        pi = partition(arr,low,high)

        quickSort(arr,low,pi-1)
        quickSort(arr,pi+1,high)

#Driver code
arr = [33,3,4]
n = len(arr)

print("The orignal array is : \n")
for i in range(n):
    print(arr[i])

quickSort(arr, 0, (n-1))

print("The sorted array is : \n")
for i in range(n):
    print(arr[i])
