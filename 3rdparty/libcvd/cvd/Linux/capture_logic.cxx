//Expexts video buffer valled vbuf and frame?* vf and display vd
	
	while(vd.pending())
		{
			vd.get_event(&e);
			if(e.type == ButtonPress)
			{
				delete vbuf;
				return 0;
			} 
			else if(e.type == KeyPress)
			{
				KeySym k;

				XLookupString(&e.xkey, 0, 0, &k, 0);

				if(k == XK_c)
				{
					char buf[100];
					char *name;
					
					name=strrchr(argv[0], '/');

					if(!name) 
						name=argv[0];
					else
						name++;
					
					sprintf(buf, "capture-%s-%05i.pnm", name, saved_name);

					ofstream out;

					out.open(buf);
					pnm_save(*vf, out);
					out.close();

					std::cout << "Saved: " << buf << "\n";

					saved_name++;
				}
				else if(k == XK_Escape || k == XK_q || k == XK_Q)
				{
					delete vbuf;
					return 0;
		
				}


			}

