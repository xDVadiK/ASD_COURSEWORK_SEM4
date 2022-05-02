#pragma once
#include <iostream>

using namespace std;

enum Color_Tree_Node
{
	red = 0,
	black = 1,
};

// Class Red Black Tree
template<typename T1, typename T2>
class Red_Black_Tree
{
private:

	// Red Black tree element class
	class Element_Red_Black_Tree
	{
	public:

		int color; // Node color
		T1* key; // Node Key
		T2* value; // Node Value
		Element_Red_Black_Tree* left_node; // Pointer to the left child of the node
		Element_Red_Black_Tree* right_node; // Pointer to the right child of the node
		Element_Red_Black_Tree* parent_node; // Pointer to the node's parent

		// Constructor of the Red Black tree element class
		Element_Red_Black_Tree(int color = red, T1* key = nullptr, T2* value = nullptr, Element_Red_Black_Tree* parent_node = nullptr, Element_Red_Black_Tree* left_node = nullptr, Element_Red_Black_Tree* right_node = nullptr)
		{
			this->color = color;
			this->key = key;
			this->value = value;
			this->parent_node = parent_node;
			this->left_node = left_node;
			this->right_node = right_node;
		}
		// The destructor of the Red Black tree element class
		~Element_Red_Black_Tree()
		{
			color = 0;
			key = nullptr;
			value = nullptr;
			parent_node = nullptr;
			left_node = nullptr;
			right_node = nullptr;
		}

	};

	size_t size_tree; // The size of the red and black wood
	Element_Red_Black_Tree* root_node; // Pointer to the root node
	Element_Red_Black_Tree* nil; // Pointer to a leaf (bounding) node

public:

	// Constructor of the Red Black Tree class
	Red_Black_Tree()
	{
		size_tree = 0;
		root_node = nullptr;
		nil = new Element_Red_Black_Tree(black);
	}
	// The destructor of the Red Black Tree class
	~Red_Black_Tree()
	{
		clear();
		size_tree = 0;
		root_node = nullptr;
		nil = nullptr;
	}

	// Getting a pointer to the root element of the red-black tree
	Element_Red_Black_Tree* get_root_node()
	{
		if (get_size_tree() != 0)
		{
			return root_node;
		}
		else throw domain_error("Red-black tree (associative array) is empty");
	};

	// Getting the size of a red-black tree
	size_t get_size_tree()
	{
		return size_tree;
	}

	// Getting a pointer to a leaf node of a red-black tree
	Element_Red_Black_Tree* get_nil_node()
	{
		if (get_size_tree() != 0)
		{
			return nil;
		}
		else throw domain_error("Red-black tree (associative array) is empty");
	}

	// Adding an element with a key and value
	void insert(T1 key, T2 value)
	{
		if (get_size_tree() != 0)
		{
			bool New_element_added(false);
			Element_Red_Black_Tree* current_element = root_node;
			while (!New_element_added)
			{
				if (key > *current_element->key && (current_element->right_node == nullptr || current_element->right_node == nil))
				{
					current_element->right_node = new Element_Red_Black_Tree(red, new T1(key), new T2(value), current_element);
					current_element->right_node->left_node = nil;
					current_element->right_node->right_node = nil;
					Restoring_properties_insert(current_element->right_node); // Restoring the properties of the red-black tree
					New_element_added = true;
					size_tree++;
				}
				else if (key < *current_element->key && (current_element->left_node == nullptr || current_element->left_node == nil))
				{
					current_element->left_node = new Element_Red_Black_Tree(red, new T1(key), new T2(value), current_element);
					current_element->left_node->left_node = nil;
					current_element->left_node->right_node = nil;
					Restoring_properties_insert(current_element->left_node); // Restoring the properties of the red-black tree
					New_element_added = true;
					size_tree++;
				}
				else if (key > *current_element->key)
				{
					current_element = current_element->right_node;
				}
				else if (key < *current_element->key)
				{
					current_element = current_element->left_node;
				}
				else if (key == *current_element->key)
				{
					throw domain_error("An associative array cannot contain elements with the same keys");
				}
			}
		}
		else
		{
			root_node = new Element_Red_Black_Tree(black, new T1(key), new T2(value));
			if (nil == nullptr) nil = new Element_Red_Black_Tree(black);
			root_node->left_node = nil;
			root_node->right_node = nil;
			root_node->parent_node = nil;
			nil->left_node = root_node;
			nil->right_node = root_node;
			size_tree++;
		}
	};

	// Left turn for red and black wood elements
	void Left_turn(Element_Red_Black_Tree* element_x)
	{
		Element_Red_Black_Tree* element_y = element_x->right_node;
		// If the node (element_y) has a left subtree
		if (element_y->left_node != nullptr)
		{
			element_x->right_node = element_y->left_node;
			if (element_y->left_node != nil) element_y->left_node->parent_node = element_x;
			element_y->left_node = nullptr;
			element_y->parent_node = nullptr;
		};
		// If the parent of the node (element_x) is NULL
		if (element_x->parent_node == nil)
		{
			root_node = element_y;
			element_y->parent_node = nil;
			nil->left_node = element_y;
			nil->right_node = element_y;
		}
		// if node (element_x) is the left child
		else if (element_x->parent_node->left_node == element_x)
		{
			element_x->parent_node->left_node = element_y;
			element_y->parent_node = element_x->parent_node;
		}
		else
		{
			element_x->parent_node->right_node = element_y;
			element_y->parent_node = element_x->parent_node;
		}
		element_x->parent_node = element_y;
		element_y->left_node = element_x;
	}

	// Right turn for red and black wood elements
	void Right_turn(Element_Red_Black_Tree* element_y)
	{
		Element_Red_Black_Tree* element_x = element_y->left_node;
		// If the node (element_x) has a right subtree
		if (element_x->right_node != nullptr)
		{
			element_y->left_node = element_x->right_node;
			if (element_x->right_node != nil) element_x->right_node->parent_node = element_y;
			element_x->right_node = nullptr;
			element_x->parent_node = nullptr;
		};
		// If the parent of the node (element_y) is NULL
		if (element_y->parent_node == nil)
		{
			root_node = element_x;
			element_x->parent_node = nil;
			nil->left_node = element_x;
			nil->right_node = element_x;
		}
		// if node (element_y) is the right child
		else if (element_y->parent_node->right_node == element_y)
		{
			element_y->parent_node->right_node = element_x;
			element_x->parent_node = element_y->parent_node;
		}
		else
		{
			element_y->parent_node->left_node = element_x;
			element_x->parent_node = element_y->parent_node;
		}
		element_y->parent_node = element_x;
		element_x->right_node = element_y;
	};

	// Restoring the properties of the red-black tree (insert)
	void Restoring_properties_insert(Element_Red_Black_Tree* new_node)
	{
		// We perform the following actions while the parent (parent_node) of the node (new_node) is red
		while (new_node->parent_node->color == red && new_node != root_node)
		{
			// If the parent (parent_node) is the left child grand parent of the inserted node (new_node)
			if (new_node->parent_node->parent_node->left_node == new_node->parent_node)
			{
				// Case 1. If the color of the right child of the grand parent node (new_node) is red
				if (new_node->parent_node->parent_node->right_node->color == red)
				{
					new_node->parent_node->parent_node->right_node->color = black;
					new_node->parent_node->color = black;
					new_node->parent_node->parent_node->color = red;
					new_node = new_node->parent_node->parent_node;
				}
				// Case 2. If node (new_node) is the right child of (parent_node)
				else if (new_node->parent_node->right_node == new_node)
				{
					new_node = new_node->parent_node;
					Left_turn(new_node); // Left turn for red and black wood elements
				}
				// Case 3.
				else
				{
					new_node->parent_node->color = black;
					new_node->parent_node->parent_node->color = red;
					Right_turn(new_node->parent_node->parent_node); // Right turn for red and black wood elements
				}
			}
			// If the parent (parent_node) is the right child grand parent of the inserted node (new_node)
			else if (new_node->parent_node->parent_node->right_node == new_node->parent_node)
			{
				// Case 1. If the color of the left child of the grand parent node (new_node) is red
				if (new_node->parent_node->parent_node->left_node->color == red)
				{
					new_node->parent_node->parent_node->left_node->color = black;
					new_node->parent_node->color = black;
					new_node->parent_node->parent_node->color = red;
					new_node = new_node->parent_node->parent_node;
				}
				// Case 2. If node (new_node) is the left child of (parent_node)
				else if (new_node->parent_node->left_node == new_node)
				{
					new_node = new_node->parent_node;
					Right_turn(new_node); // Right turn for red and black wood elements
				}
				// Case 3.
				else
				{
					new_node->parent_node->color = black;
					new_node->parent_node->parent_node->color = red;
					Left_turn(new_node->parent_node->parent_node); // Left turn for red and black wood elements
				}
			}
		}
		root_node->color = black;
	};

	// Search for an element by key in a red-black tree
	bool find(T1 key)
	{
		if (get_size_tree() != 0)
		{
			bool Element_found(false);
			Element_Red_Black_Tree* current_element = root_node;
			while (!Element_found)
			{
				if (key == *current_element->key)
				{
					return true;
				}
				else if (key > *current_element->key)
				{
					current_element = current_element->right_node;
				}
				else if (key < *current_element->key)
				{
					current_element = current_element->left_node;
				}
				if (current_element == nil)
				{
					return false;
				}
			}
		}
		else
		{
			return false;
		}
	};

	// Getting a red-black tree element
	Element_Red_Black_Tree* Get_map_element(T1 key)
	{
		if (get_size_tree() != 0 && find(key))
		{
			bool Element_found(false);
			Element_Red_Black_Tree* current_element = root_node;
			while (!Element_found)
			{
				if (key == *current_element->key)
				{
					return current_element;
				}
				else if (key > *current_element->key)
				{
					current_element = current_element->right_node;
				}
				else if (key < *current_element->key)
				{
					current_element = current_element->left_node;
				}
			}
		}
		else throw domain_error("The element could not be found");
	};

	// Deleting a red-black tree element by key
	void remove(T1 key)
	{
		if (find(key))
		{
			bool Element_removed(false);
			Element_Red_Black_Tree* delete_node = root_node;
			while (!Element_removed)
			{
				if (key == *delete_node->key)
				{
					int original_color = delete_node->color;
					// If the node being deleted is the only one in the red black tree
					if (delete_node->left_node == nil && delete_node->right_node == nil && delete_node == root_node)
					{
						nil->left_node = nullptr;
						nil->right_node = nullptr;
						delete delete_node;
						Element_removed = true;
						size_tree = 0;
					}
					else
					{
						// If the left child of the node being deleted (delete_node) is NULL (nil)
						if (delete_node->left_node == nil)
						{
							// Denote the right child of the node to be Deleted (delete_node) for element_x
							Element_Red_Black_Tree* element_x = delete_node->right_node;
							if (delete_node == root_node)
							{
								root_node = element_x;
								nil->left_node = element_x;
								nil->right_node = element_x;
							}
							// Moving element_x to the place of the node to be deleted (delete_node)
							if (delete_node->parent_node->left_node == delete_node)
								delete_node->parent_node->left_node = element_x;
							else delete_node->parent_node->right_node = element_x;
							element_x->parent_node = delete_node->parent_node;
							delete delete_node;
							Element_removed = true;
							size_tree--;
							// Restoring the properties of the red-black tree after removal
							if (original_color == black) Restoring_properties_remove(element_x);
						}
						// If the right child of the node being deleted (delete_node) is NULL (nil)
						else if (delete_node->right_node == nil)
						{
							Element_Red_Black_Tree* element_x = delete_node->left_node;
							if (delete_node == root_node)
							{
								root_node = element_x;
								nil->left_node = element_x;
								nil->right_node = element_x;
							}
							// Moving element_x to the place of the node to be deleted (delete_node)
							if (delete_node->parent_node->left_node == delete_node)
								delete_node->parent_node->left_node = element_x;
							else delete_node->parent_node->right_node = element_x;
							element_x->parent_node = delete_node->parent_node;
							delete delete_node;
							Element_removed = true;
							size_tree--;
							// Restoring the properties of the red-black tree after removal
							if (original_color == black) Restoring_properties_remove(element_x);
						}
						// If both child elements of the node being deleted (delete_node) are NULL (nil)
						else
						{
							Element_Red_Black_Tree* element_y = delete_node->right_node;
							// Denote by element_y the leftmost element in the right subtree
							while (element_y->left_node != nil) element_y = element_y->left_node;
							if (delete_node == root_node)
							{
								root_node = element_y;
								nil->left_node = element_y;
								nil->right_node = element_y;
							}
							original_color = element_y->color;
							// Denote for element_x the right descendant of element_y
							Element_Red_Black_Tree* element_x = element_y->right_node;
							// If node (element_y) is a child node to be deleted (delete_node)
							if (element_y->parent_node == delete_node)
							{
								element_x->parent_node = element_y;
							}
							else
							{
								element_y->parent_node->left_node = element_x;
								element_x->parent_node = element_y->parent_node;
								element_y->right_node = delete_node->right_node;
								delete_node->right_node->parent_node = element_y;
							}
							// Moving element_y to the place of the node to be deleted (delete_node)
							if (delete_node->parent_node != nil)
							{
								if (delete_node->parent_node->left_node == delete_node)
									delete_node->parent_node->left_node = element_y;
								else delete_node->parent_node->right_node = element_y;
							}
							element_y->parent_node = delete_node->parent_node;
							element_y->left_node = delete_node->left_node;
							delete_node->left_node->parent_node = element_y;
							element_y->color = delete_node->color;
							delete delete_node;
							Element_removed = true;
							size_tree--;
							// Restoring the properties of the red-black tree after removal
							if (original_color == black) Restoring_properties_remove(element_x);
						}
					}
				}
				else if (key > *delete_node->key)
				{
					delete_node = delete_node->right_node;
				}
				else if (key < *delete_node->key)
				{
					delete_node = delete_node->left_node;
				}
			}
		}
		else throw domain_error("An element with such a key is missing in the array");
	}

	// Restoring the properties of the red-black tree (deletion)
	void Restoring_properties_remove(Element_Red_Black_Tree* element_x)
	{
		Element_Red_Black_Tree* brother_node;
		// Until node (element_x) becomes the root of the tree and the color (element_x) is black
		while (element_x != root_node && element_x->color == black)
		{
			// If node (element_x) is the left child
			if (element_x->parent_node->left_node == element_x)
			{
				brother_node = element_x->parent_node->right_node;
				// Case 1. If node (brother_node) is red
				if (brother_node->color == red)
				{
					brother_node->color = black;
					element_x->parent_node->color = red;
					Left_turn(element_x->parent_node); // Performing a left turn for the parent element_x
					brother_node = element_x->parent_node->right_node;
				}
				// Case 2. If the color of the right and left child of node (brother_node) is black
				if (brother_node->left_node->color == black && brother_node->right_node->color == black)
				{
					brother_node->color = red;
					element_x = element_x->parent_node;
					if (nil->parent_node != nullptr) nil->parent_node = nullptr;
				}
				else
				{
					// Case 3. If the color of the right child of node (brother_node) is black
					if (brother_node->right_node->color == black)
					{
						brother_node->left_node->color = black;
						brother_node->color = red;
						Right_turn(brother_node); // Performing a right turn for the node brother_node
						brother_node = element_x->parent_node->right_node;
					}
					// Case 4. If none of the above cases occurs
					brother_node->color = element_x->parent_node->color;
					element_x->parent_node->color = black;
					brother_node->right_node->color = black;
					Left_turn(element_x->parent_node); // Performing a left turn for the parent element_x
					nil->parent_node = nullptr;
					element_x = root_node;
				}
			}
			// If node (element_x) is the right child
			else if (element_x->parent_node->right_node == element_x)
			{
				brother_node = element_x->parent_node->left_node;
				// Case 1. If node (brother_node) is red
				if (brother_node->color == red)
				{
					brother_node->color = black;
					element_x->parent_node->color = red;
					Right_turn(element_x->parent_node); // Performing a right turn for the parent element_x
					brother_node = element_x->parent_node->left_node;
				}
				// Case 2. If the color of the right and left child of node (brother_node) is black
				if (brother_node->left_node->color == black && brother_node->right_node->color == black)
				{
					brother_node->color = red;
					element_x = element_x->parent_node;
					if (nil->parent_node != nullptr) nil->parent_node = nullptr;
				}
				else
				{
					// Case 3. If the color of the left child of node (brother_node) is black
					if (brother_node->left_node->color == black)
					{
						brother_node->right_node->color = black;
						brother_node->color = red;
						Left_turn(brother_node); // Performing a left turn for the node brother_node
						brother_node = element_x->parent_node->left_node;
					}
					// Case 4. If none of the above cases occurs
					brother_node->color = element_x->parent_node->color;
					element_x->parent_node->color = black;
					brother_node->left_node->color = black;
					Right_turn(element_x->parent_node); // Performing a right turn for the parent element_x
					nil->parent_node = nullptr;
					element_x = root_node;
				}
			}
		}
		element_x->color = black;
	}

	// Clearing the associative array (removing all elements of the red-black tree)
	void clear()
	{
		if (get_size_tree() != 0)
		{
			while (get_size_tree() != 0)
			{
				remove(*root_node->key); // Removing the root element of the red-black tree
			}
			delete nil;
			nil = nullptr;
			root_node = nullptr;
			size_tree = 0;
		}
	}

	// Class Queue
	class Queue
	{
		// Class Queue element
		class Queue_element
		{
		public:

			Element_Red_Black_Tree* data; // Pointer to the node of the red - black tree
			Queue_element* next; // Pointer to the next queue element

			// Constructor of the queue element class
			Queue_element(Element_Red_Black_Tree* data = nullptr, Queue_element* next = nullptr)
			{
				this->data = data;
				this->next = next;
			}
			// The destructor of the queue element class
			~Queue_element()
			{
				data = nullptr;
				next = nullptr;
			}
		};

		Queue_element* head; // Pointer to the beginning of the queue
		Queue_element* tail; // Pointer to the end of the queue
		size_t size_queue; // Queue size

	public:

		// Queue Class Constructor
		Queue()
		{
			head = nullptr;
			tail = nullptr;
			size_queue = 0;
		}
		// Queue class destructor
		~Queue()
		{
			clear_queue();
		};

		// Checking the queue for emptiness
		bool queue_is_empty()
		{
			return size_queue == 0;
		}

		// Getting the queue size
		int get_size_queue()
		{
			return size_queue;
		}

		// Getting a pointer to the head element of the queue
		Queue_element* get_head()
		{
			return head;
		}

		// Getting a pointer to the tail element of the queue
		Queue_element* get_tail()
		{
			return tail;
		}

		// Adding an element to the queue
		void enqueue(Element_Red_Black_Tree* data)
		{
			Queue_element* current = new Queue_element(data);
			if (queue_is_empty())
			{
				head = current;
				tail = current;
				size_queue++;
			}
			else
			{
				tail->next = current;
				tail = current;
				size_queue++;
			}
		};

		// Removing an element from the queue
		Element_Red_Black_Tree* dequeue()
		{
			if (!queue_is_empty())
			{
				Element_Red_Black_Tree* element;
				Queue_element* delete_element;
				if (size_queue == 1)
				{
					element = head->data;
					delete head;
					head = nullptr;
					tail = nullptr;
					size_queue = 0;
					return element;
				}
				else
				{
					element = head->data;
					delete_element = head;
					head = head->next;
					delete delete_element;
					size_queue--;
					return element;
				}
			}
			else return nullptr;
		}

		// Clearing the queue
		void clear_queue()
		{
			if (!queue_is_empty())
			{
				while (!queue_is_empty())
				{
					dequeue();
				}
				head = nullptr;
				tail = nullptr;
				size_queue = 0;
			}
		}
	};

	// Iterator class of tree traversal in width
	class Breadth_first_traverse_iterator
	{
	private:

		Element_Red_Black_Tree* nil_node; // Pointer to the leaf (bounding) node of the red-black tree
		Element_Red_Black_Tree* current; // Pointer to a red-black tree element
		Queue queue; // Creating a queue to implement a tree-wide traversal

	public:

		// Constructor of the tree-wide traversal class
		Breadth_first_traverse_iterator(Element_Red_Black_Tree* root_node, Element_Red_Black_Tree* nil)
		{
			current = root_node;
			queue.enqueue(current);
			nil_node = nil;
		};
		// The destructor of the tree-wide traversal class
		~Breadth_first_traverse_iterator()
		{
			current = nullptr;
			queue.clear_queue();
		};

		// Processing of the current node by a tree-wide traversal algorithm
		Element_Red_Black_Tree* next()
		{
			if (has_next())
			{
				current = queue.dequeue();
				if (current->left_node != nil_node)
				{
					queue.enqueue(current->left_node);
				}
				if (current->right_node != nil_node)
				{
					queue.enqueue(current->right_node);
				}
				return current;
			}
			else throw domain_error("There are no tree elements to process");
		};

		// Checking for the presence of elements not bypassed by the iterator of traversing the tree in width
		bool has_next()
		{
			return !queue.queue_is_empty();
		};

	};

	// List class
	template<typename T>
	class List
	{
	private:

		// List node class
		class Node
		{
		public:

			T data; // The data of the list item
			Node* next; // Pointer to the next item in the list

			// Constructor of the List node class
			Node(T data = 0, Node* next = nullptr) {
				this->data = data;
				this->next = next;
			};
			// List node class destructor
			~Node()
			{
				data = 0;
				next = nullptr;
			};
		};

		size_t size_list; // List size
		Node* head; // Pointer to the first item in the list
		Node* last; // Pointer to the last item in the list

	public:

		// Constructor of the parent class
		List(size_t size = 0, Node* head = nullptr, Node* last = nullptr)
		{
			this->size_list = size;
			this->head = head;
			this->last = last;
		}
		// Parent class destructor
		~List()
		{
			clear_list();
			size_list = 0;
			head = nullptr;
			last = nullptr;
		};

		// Checking the list for emptiness
		bool isEmpty()
		{
			return (size_list == 0);
		};

		// Getting the list size
		size_t get_size_list()
		{
			return size_list;
		}

		// Getting the head element of the list
		Node* get_head()
		{
			return head;
		}

		// Adding a new item to the list
		void push_back(T element_value)
		{
			if (head == nullptr)
			{
				Node* new_list_item = new Node(element_value, nullptr);
				head = new_list_item;
				last = new_list_item;
				size_list++;
			}
			else
			{
				Node* new_list_item = new Node(element_value, nullptr);
				last->next = new_list_item;
				last = new_list_item;
				size_list++;
			}
		};

		// Deleting the first item in the list
		void pop_front()
		{
			if (get_size_list() > 0)
			{
				if (get_size_list() == 1)
				{
					clear_list();
				}
				else
				{
					Node* list_item = head;
					head = head->next;
					delete list_item;
					size_list--;
				}
			}
			else throw invalid_argument("The list is empty, there are no available items to delete");
		};

		// Deleting all list items
		void clear_list()
		{
			if (!isEmpty())
			{
				Node* current_item = head;
				while (current_item != nullptr)
				{
					current_item = current_item->next;
					delete head;
					head = current_item;
				}
				head = nullptr;
				last = nullptr;
				size_list = 0;
			}
		};

		// Overloading the output operator
		friend ostream& operator<<(ostream& stream, const List<T>& List)
		{
			if (List.head != nullptr)
			{
				List::Node* current = List.head;
				while (current != nullptr)
				{
					stream << current->data << " ";
					current = current->next;
				}
				stream << '\n';
				return stream;
			}
			else return stream << "The list is empty\n\n";
		};
	};

	// Getting a list of keys of an associative array (red-black tree)
	List<T1> get_keys()
	{
		if (get_size_tree() != 0)
		{
			List<T1>* List_keys = new List<T1>;
			Breadth_first_traverse_iterator* bft_iterator = new Breadth_first_traverse_iterator(root_node, nil);
			while (bft_iterator->has_next()) List_keys->push_back(*bft_iterator->next()->key);
			return *List_keys;
		}
		else throw domain_error("Red-black tree (associative array) is empty");
	}

	// Getting a list of values of an associative array (red-black tree)
	List<T2> get_values()
	{
		if (get_size_tree() != 0)
		{
			List<T2>* List_value = new List<T2>;
			Breadth_first_traverse_iterator* bft_iterator = new Breadth_first_traverse_iterator(root_node, nil);
			while (bft_iterator->has_next()) List_value->push_back(*bft_iterator->next()->value);
			return *List_value;
		}
		else throw domain_error("Red-black tree (associative array) is empty");
	}

	// Output of an associative array(red-black tree) to the console
	void print()
	{
		if (get_size_tree() != 0)
		{
			Breadth_first_traverse_iterator* bft_iterator = new Breadth_first_traverse_iterator(root_node, nil);
			Element_Red_Black_Tree* Node;
			while (bft_iterator->has_next())
			{
				Node = bft_iterator->next();
				cout << "Key: " << *Node->key << " | Value: " << *Node->value << "\n";
			}
		}
		else throw domain_error("Red-black tree (associative array) is empty");
	}
};