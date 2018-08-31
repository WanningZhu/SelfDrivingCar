#include "an_dynamic_planner/minHeap.h"


MinHeap::MinHeap(){
	heap_size = 0;
}
//Get left child index of the element in position i
int MinHeap::leftChild(int i){
	return i*2+1;
}
//Get right child index of the element in position i
int MinHeap::rightChild(int i){
	return i*2+2;
}
//Get the parent index of the element in position i
int MinHeap::parent(int i){
	return (i-1)/2;
}
bool MinHeap::find(mapCell* k){
	for(int i = 0; i < heap_size; i++){
		if(heap[i] == k){
			return true;
		}
	}
	return false;
}
void MinHeap::push(mapCell* cell_pointer){
	heap.push_back(cell_pointer);
	heap_size++;
	if(heap_size == 1){
		return;
	}
	compareParent(heap_size-1);
}


mapCell* MinHeap::peekTop(){
	if(heap_size == 0){
		ROS_ERROR("[MH] PeekTop: Empty heap!");
		return NULL;
	}
	return heap[0];
}



mapCell* MinHeap::popTop(){
	if(heap_size == 0){
		ROS_ERROR("[MH] PopTop: Empty heap!");
		return NULL;
	}
	mapCell* top = heap[0];
	if(heap_size == 1){
		heap_size--;
		heap.pop_back();
		return top;
	}
	swap(0, heap_size-1);
	heap.pop_back();
	heap_size--;
	compareChild(0);
	return top;
}

void MinHeap::update(mapCell* k){
	for(int i = 0; i < heap_size; i++){
		if(heap[i] == k){
			while(i != 0 && heap[parent(i)]->compareF(heap[i]) == 1){
				swap(i, parent(i));
				i = parent(i);
			}
			return;
		}
	}
	ROS_ERROR("[MH] decreaseValueKey: Can't find Key!");
}
void MinHeap::empty(){
	heap.empty();
}
void MinHeap::compareChild(int i){
	int l = leftChild(i);
	int r = rightChild(i);
	int smallest = i;
	if(l < heap_size && heap[l]->compareF(heap[i]) == -1){
		smallest = l;
	}
	if(r < heap_size && heap[r]->compareF(heap[smallest]) == -1){
		smallest = r;
	}
	if(smallest != i){
		swap(i, smallest);
		compareChild(smallest);
	}
}

void MinHeap::compareParent(int i){
	while(i != 0 && heap[parent(i)]->compareF(heap[i]) == 1){
		swap(i, parent(i));
		i = parent(i);
	}
}


void MinHeap::swap(int i, int j){
	mapCell* temp = heap[i];
	heap[i] = heap[j];
	heap[j] = temp;
}
bool MinHeap::isEmpty(){
	if(heap.size() == 0){
		return true;
	}
	return false;
}
void MinHeap::drawHeap(){
	int i = 1;
	int j = 0;
	while(1){
		for(int k = 0; k < i; k++){
			if(j >= heap_size){
std::cout << std::endl;
				return;
			}
			std::cout << heap[j]->f << "  ";
			j++;
		}
	std::cout << "std" << i << '\n';
	i = i*2;
	std::cout << std::endl;	
	}
	std::cout << std::endl;
}
