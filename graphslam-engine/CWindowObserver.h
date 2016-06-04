#ifndef CWINDOWOBSERVER_H
#define CWINDOWOBSERVER_H

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/utils/CObserver.h>

#include <iostream>

class CWindowObserver : public mrpt::utils::CObserver
{
public:
	CWindowObserver() :
		m_mouse_clicked(false) {
		std::cout << "WindowObserver initialized." << endl;
	}
	bool clickedMouseBtn() { return m_mouse_clicked; }

	virtual void OnEvent(const mrptEvent &e) {
		if (e.isOfType<mrptEventOnDestroy>()) {
			const mrptEventOnDestroy &ee = static_cast<const mrptEventOnDestroy &>(e);
			//std::cout  << "Event received: mrptEventOnDestroy" << endl;
		}
		else if (e.isOfType<mrptEventWindowResize>()) {
			const mrptEventWindowResize &ee = static_cast<const mrptEventWindowResize &>(e);
			//std::cout  << "Resize event received from: " << ee.source_object
				//<< ", new size: " << ee.new_width << " x " << ee.new_height << endl;
		}
		else if (e.isOfType<mrptEventWindowChar>()) {
			const mrptEventWindowChar &ee = static_cast<const mrptEventWindowChar &>(e);
			//std::cout  << "Char event received from: " << ee.source_object
				//<< ". Char code: " <<  ee.char_code << " modif: " << ee.key_modifiers << endl;;
		}
		else if (e.isOfType<mrptEventWindowClosed>()) {
			const mrptEventWindowClosed &ee = static_cast<const mrptEventWindowClosed &>(e);
			//std::cout  << "Window closed event received from: "
				//<< ee.source_object<< "\n";
		}
		else if (e.isOfType<mrptEventMouseDown>()) {
			const mrptEventMouseDown &ee = static_cast<const mrptEventMouseDown&>(e);
			m_mouse_clicked = true;

			//std::cout  << "Mouse down event received from: "
				//<< ee.source_object<< "pt: " <<ee.coords.x << "," << ee.coords.y << "\n";
			//std::cout  << "rightButton: " << ee.rightButton << endl;
		}
		else {
			//std::cout  << "Unregistered mrptEvent received\n";
		}
	}

private:
	bool m_mouse_clicked;

};


#endif /* end of include guard: CWINDOWOBSERVER_H */
