//
//  ViewController.m
//  CVApps
//
//  Created by Aishanou Osha Rait on 11/30/16.
//  Copyright © 2016 Aishanou Osha Rait. All rights reserved.
//


#import "ViewController.h"
#import "GetFramesViewController.h"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#import "opencv2/imgcodecs/ios.h"
#import <opencv2/core/core.hpp>
#import <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <stdio.h>
#include <fstream>





using namespace std;

@interface ViewController (){
    UIImageView *imageView_;
}

@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
 
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


// When the movie is done, release the controller.
-(void)myMovieFinishedCallback:(NSNotification*)aNotification {
    cout << "Executing myMovieFinishedCallBack of main ViewController" << endl;
    [self dismissMoviePlayerViewControllerAnimated];
    MPMoviePlayerController* theMovie = [aNotification object];
    [[NSNotificationCenter defaultCenter] removeObserver:self
                                                    name:MPMoviePlayerPlaybackDidFinishNotification object:theMovie];
}


// For responding to the user tapping Cancel.
-(void)imagePickerControllerDidCancel:(UIImagePickerController *)picker {
    cout << "Executing imagePickerControllerDidCancel of main ViewController" << endl;
    [self dismissViewControllerAnimated: YES completion:nil];
}




- (IBAction)SelectVideo:(id)sender {
    [self startMediaBrowserFromViewController:self usingDelegate:self];
}

-(BOOL)startMediaBrowserFromViewController:(UIViewController*)controller usingDelegate:(id )delegate {
    cout << "Executing startMediaBrowserFromViewController of main ViewController" << endl;
    // 1 - Validations
    if (([UIImagePickerController isSourceTypeAvailable:UIImagePickerControllerSourceTypeSavedPhotosAlbum] == NO)
        || (delegate == nil)
        || (controller == nil)) {
        return NO;
    }
    // 2 - Get image picker
    UIImagePickerController *mediaUI = [[UIImagePickerController alloc] init];
    mediaUI.sourceType = UIImagePickerControllerSourceTypeSavedPhotosAlbum;
    mediaUI.mediaTypes = [[NSArray alloc] initWithObjects: (NSString *) kUTTypeMovie, nil];
    // Hides the controls for moving & scaling pictures, or for
    // trimming movies. To instead show the controls, use YES.
    mediaUI.allowsEditing = YES;
    mediaUI.delegate = delegate;
    // 3 - Display image picker
    [controller presentViewController:mediaUI animated:YES completion:nil];
    return YES;
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([[segue identifier] isEqualToString:@"ProcessVideo"])
    {
        GetFramesViewController *getFramesViewController = [segue destinationViewController];
        getFramesViewController.MyMovie = self.movie;
        //getFramesViewController.asset = asset;
        
    }
}


-(void)imagePickerController:(UIImagePickerController *)picker didFinishPickingMediaWithInfo:(NSDictionary *)info {
    cout << "Executing imagePickerController of main ViewController" << endl;;
    // 1 - Get media type
    NSString *mediaType = [info objectForKey: UIImagePickerControllerMediaType];
    // 2 - Dismiss image picker
    [self dismissViewControllerAnimated:NO completion:nil];
    // Handle a movie capture
    if (CFStringCompare ((__bridge_retained CFStringRef)mediaType, kUTTypeMovie, 0) == kCFCompareEqualTo) {
        //3 - Get Asset
        AVURLAsset *asset = [[AVURLAsset alloc] initWithURL:[info objectForKey:UIImagePickerControllerMediaURL] options:nil];
        self.movie = asset;
        [self performSegueWithIdentifier: @"ProcessVideo" sender: self];
        
    }
   
    
}


    
    




@end
