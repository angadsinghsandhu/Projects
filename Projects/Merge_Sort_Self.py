def merge(arr, low, m, high):
    #initialization of new subarrays
    L = []
    R = []
    n1 = int(m+1 - low)
    n2 = int(high - m)

    for i in range(0,n1):
        L.append(0)
    for i in range(0,n2):
        R.append(0)

    for i in range(0,n1):
        L[i] = arr[int(low + i)]
    for j in range(0,n2):
        R[j] = arr[int((m+1) + j)]

    #copying into new arrays
    i = 0
    j = 0
    k = 0
    while i<n1 and j<n2:
        if(L[i] <= R[j]):
            arr[k] = L[i]
            i+=1
        else:
            arr[k] = R[j]
            j+=1
        k+=1

    while i<n1:
        arr[k] = L[i]
        i+=1
        k+=1

    while j<n2:
        arr[k] = R[j]
        j+=1
        k+=1

def mergeSort(arr, low, high):
    if low<high:
        m = int((low+(high-1))/2)

        mergeSort(arr, low, m)
        mergeSort(arr, m+1, high)
        merge(arr, low, m, high)

#Driver Code
arr = [22,13,34,51,16,7,8]
n = len(arr)

print("orignal Array is : \n")
for i in range(n):
    print(arr[i],"\t")

mergeSort(arr,0,(n-1))

print("Sorted Array is : \n")
for i in range(n):
    print(arr[i],"\t")
