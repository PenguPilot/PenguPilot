/* Very simple implementation of a Median Filter (window)
Data needs to be stored outside;
Not optimal performance;
To find a median value, qsort algorithm can be executed only for a first half of an array;
The returned median value isn't really accurate (int)
*/

//Simple implementation of qsort algorithm
void q_sort(int *array_original, int size, int *final_array)
{
 int i, j, pos_min, temp;
 int array[size];

 for (i=0; i < size; i++)
    {
     array[i] = *(array_original + i);
    }

 for (i=0; i < (size - 1); i++)
    {
     pos_min = i;

     for (j=i+1; j < size; j++)
     {
      if (array[j] < array[pos_min])
        {
	 pos_min = j;
        }
     }

     if (pos_min != i)
       {
	temp = array[pos_min];
     	array[pos_min] = array[i];
     	array[i] = temp;
       }
    }

 for (i=0; i < size; i++)
    {
     *(final_array+i) = array[i];
    }
}

//find the median of a given array
int findMedian(int *array_original, int size)
{
 float center;
 int pos;
 int final_array[size];

 //sort a given array
 q_sort(array_original, size, final_array);

 //find a center value of an array (not precise)
 center = (size - 1) / 2.0;

 pos = (int)center;

 //return the median value
 return final_array[pos];
}

/*Window Median Filter*/

//Init a Median Filter
void filter_init(int *window, int windowSize, int *current_pos)
{
 //set current position to 0
 *current_pos = 0;

 //fill the given array(window) with zeros
 for(int i = 0; i < windowSize; i++)
  {
   *(window + i) = 0;
  }
}

//Update a Median Filter with a new value; return value is the median from a current window
float filter_upd(int *window, int windowSize, int *current_pos, int newValue)
{  
 //put a new value in the given position
 *(window+*current_pos) = newValue;
 
 //update the current position
 if (*current_pos < windowSize - 1)
   *current_pos += 1;
 else
   *current_pos = 0;

 //find the median of a current window
 float median = findMedian(window, windowSize);

 return median;
}


