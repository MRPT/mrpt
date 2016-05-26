// Thu May 26 22:24:56 EEST 2016, nickkouk
// used for testing whether the EdgeCounter class works as expected
// function for testing purposes
    void testEdgeCounterObject() {
      MRPT_START

      // create instance
      EdgeCounter_t edge_counter(m_win);

      int a;
      edge_counter.addEdgeType("odometry");
      edge_counter.getNumForEdgeType("odometry", &a);
      cout << "a = " << a << endl;

      int b = edge_counter.getNumForEdgeType("odometry");
      cout << "b = " << a << endl;

      // this throws correctly - already initialized
      //edge_counter.addEdge("odometry", /* is_loop_closure = */ false, /*is_new = */ true);

      // this throws correctly - both true
      //edge_counter.addEdge("odometry", /* is_loop_closure = */ true, /*is_new = */ true);

      // adding / removing edges of type odometry
      cout << "Total Number of edges: " << edge_counter.getTotalNumOfEdges() << endl;
      cout << "Odometry edges: " << edge_counter.getNumForEdgeType("odometry") << endl;
      edge_counter.addEdge("odometry");
      cout <<  "Odometry edges after another addition: "<< edge_counter.getNumForEdgeType("odometry") << endl;
      cout << "Total Number of edges: " << edge_counter.getTotalNumOfEdges() << endl;


      // works correctly
      // initialize ICP constraints using addEdgeType method
      edge_counter.addEdgeType("ICP");
      edge_counter.addEdge("ICP");
      edge_counter.addEdge("ICP");
      edge_counter.addEdge("ICP");
      edge_counter.addEdge("ICP");
      cout << "Total Number of edges: " << edge_counter.getTotalNumOfEdges() << endl;
      cout << "Odometry edges: " << edge_counter.getNumForEdgeType("odometry") << endl;
      cout << "ICP edges: " << edge_counter.getNumForEdgeType("ICP") << endl;


      // 3rd type of edge.
      // works correctly
      edge_counter.addEdgeType("Visual");
      edge_counter.addEdge("Visual");
      edge_counter.addEdge("Visual", /* is_loop_closure */ true);
      edge_counter.addEdge("Visual", /* is_loop_closure */ true);
      edge_counter.addEdge("Visual");

      // works correctly
      edge_counter.printEdgesSummary();
      
      // works correctly
      //edge_counter.clearAllEdges();
      //edge_counter.printEdgesSummary();
      
      // setTextMessageParams usage

      // edges - general text
      string edges_text = "Edges: ";
      double offset_y_edges;
      int text_index_edges;
      this->assignTextMessageParameters(&offset_y_edges, &text_index_edges);
      m_win->addTextMessage(5,-offset_y_edges, 
          edges_text,
          TColorf(1.0, 1.0, 1.0),
          m_font_name, m_font_size, // font name & size
          mrpt::opengl::NICE,
          /* unique_index = */ text_index_edges );

      // build each one of the others
      map<string, double> name_to_offset_y;
      map<string, int> name_to_text_index;
      const char* strings[] = {"odometry", "ICP", "Visual"};
      vector<string> vec_strings(strings, strings + 3);
      for (vector<string>::const_iterator it = vec_strings.begin(); it != vec_strings.end();
          ++it) {
        this->assignTextMessageParameters(&name_to_offset_y[*it], &name_to_text_index[*it]);
        cout << "in testEdgeCounterObject: " << endl;
        cout << "name: " << *it << " | offset_y: " << name_to_offset_y[*it] << " | text_index: " << name_to_text_index[*it] << endl;
      }
      edge_counter.setTextMessageParams(name_to_offset_y, name_to_text_index, 
          m_font_name, m_font_size);
      edge_counter.updateTextMessages();

      edge_counter.addEdge("Visual", /* is_loop_closure */ true);
      edge_counter.addEdge("Visual");
      edge_counter.printEdgesSummary();
      edge_counter.updateTextMessages();

      MRPT_END
    }
