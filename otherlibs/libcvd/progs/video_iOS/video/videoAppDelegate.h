//
//  videoAppDelegate.h
//  video
//
//  Created by Gerhard Reitmayr on 07/04/2011.
//  Copyright 2011 TU Graz. All rights reserved.
//

#import <UIKit/UIKit.h>

@class videoViewController;

@interface videoAppDelegate : NSObject <UIApplicationDelegate> {

}

@property (nonatomic, retain) IBOutlet UIWindow *window;

@property (nonatomic, retain) IBOutlet videoViewController *viewController;

@end
