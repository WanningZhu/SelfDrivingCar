#ifndef MINHEAP_H
#define MINHEAP_H

#include <vector>
#include <iostream>
#include "an_dynamic_planner/mapCell.h"
#include "ros/ros.h"

class MinHeap{
public:
	std::vector<mapCell*> heap; // Priority queue container.
	int heap_size;

	//Constructor
	MinHeap();
	//Get left child of ith node in heap
	int leftChild(int i);
	//Get right child index of ith node in heap
	int rightChild(int i);
	//Get the parent index of ith node in heap
	int parent(int i);
	//Return true when find the target key in heap, otherwise return false
	bool find(mapCell* k);
	//Inset a key into the heap and maintain the min heap
	void push(mapCell* cell_pointer);
	//Seek the top element in the minheap
	mapCell* peekTop();
	//Pop the top element in the minheap
	mapCell* popTop();
	//Update K if k's value decreases
	void update(mapCell* k);
	//Return ture if the heap is empty, otherwise return false.
	bool isEmpty();
	//Clean up the heap
	void empty();
	void drawHeap();

private:
	//Swap ith node and jth node in heap
	void swap(int i, int j);
	//Compare ith node with its parent, if its f value is smaller, swap with the parent node.
	void compareParent(int i);
	//Compare with ith node with its children, if its f value is bigger, swap with the smaller child.
	void compareChild(int i);




};


#endif