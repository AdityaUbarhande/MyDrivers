#include <stdio.h>
//Write solution code below

//This code checks if the array element present at array[i] is actually the "leader" element. If it is, then flag is set to one. This is done 
//to keep myself sane and simplify the further code. 

void findLeaders(int *arr, int n)       //Predefined function. Already given by IITM
{
	int array[n];       //Copy of the array we received through the function call.
	
	//This for loop simple makes a copy of the array received through function call. The copy is stored in the variable array[n]
	for(int j = 0; j < n; j++)
	{
		array [j] = (*((arr + j)));     //Copying of contents from one array to another.. by dereferencing. 
	}
    
    char flag = 0;      //This flag is set to zero initially. If it is set to one initially, then we would get wrong leader elements.
    
    
    //The below for loop is nested. 
    
    //For the first for loop (int i wala) apan basically array[i] wala element ghenare.. And then compare that with all elements
    //to it's right side. 
    
    //Consider a array {1,2,3,4}
    //So, array[i] = 1 and array[k] will be 2, then after iteration 3 and then finally 4. 
    //Then we ask is array[i] > array[k]? Where 'k's value keeps changing. So, considering i=0 (initially) the thing evaluates to: 
    //is array[0] = array[1]?
    //is array[0] = array[2]?
    //is array[0] = array[3]?......... And so on....
    
    //Then we need to move to next array element and check it. Initially we checked array[0] now we need to check array[1]. 
    //When the inner for loop executes and exits, we enter the first for loop. So, i++ happens.
    //Now, we are at array[1] (before we were at array[0])
    //Again, it evaluates to
    //is array[1] = array[2]?
    //is array[1] = array[3]?
    //is array[1] = array[4]?......... And so on.... Get it?
    
    
	for(int i = 0; i < (n-1); i++)
	{
	   
        for(int k = i+1; k < n; k++)
        {
            
            if(array[i] > array[k])
            {
                flag = 1;
            }
            else
            {
                flag = 0;
                break;              //If array[i] is smaller than even one element on its right side, then it cannot be a leader element.
                                    //So, we can just break out of the loop and move to array[i+1] i.e. the next element.
            }
        }
       
        if(flag == 1)           //When flag is one, that means that the array[i] element we checked in above steps is LEADER ELEMENT.
        {
            printf("%d\n", arr[i]);         //So, we just print it out. 
        }
	}
	
	if(flag == 0)               //If flag = 0, the array[i] element we checked was NOT A LEADER ELEMENT
	{
	    printf("None");         //So, we just print "None"
	}
}


int main()
{
    int n;
    scanf("%d",&n);
    int arr[n];
    for(int i=0;i<n;i++)
        {
            scanf("%d",&arr[i]);
        }       
    findLeaders(arr,n);
    return 0;
}
