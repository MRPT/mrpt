object formList: TformList
  Left = 462
  Top = 352
  BorderStyle = bsToolWindow
  Caption = 'Points list'
  ClientHeight = 282
  ClientWidth = 149
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  FormStyle = fsStayOnTop
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object Panel1: TPanel
    Left = 0
    Top = 240
    Width = 149
    Height = 42
    Align = alBottom
    BevelOuter = bvNone
    TabOrder = 0
    object Button1: TButton
      Left = 68
      Top = 3
      Width = 49
      Height = 17
      Caption = 'Save...'
      TabOrder = 0
      OnClick = Button1Click
    end
    object Button2: TButton
      Left = 8
      Top = 5
      Width = 49
      Height = 17
      Caption = 'Clear'
      TabOrder = 1
      OnClick = Button2Click
    end
    object Button3: TButton
      Left = 8
      Top = 24
      Width = 49
      Height = 17
      Caption = 'Draw...'
      TabOrder = 2
      OnClick = Button3Click
    end
    object Button4: TButton
      Left = 68
      Top = 23
      Width = 49
      Height = 18
      Caption = 'Load...'
      TabOrder = 3
      OnClick = Button4Click
    end
  end
  object mem: TMemo
    Left = 0
    Top = 0
    Width = 149
    Height = 240
    Align = alClient
    ScrollBars = ssBoth
    TabOrder = 1
    WantReturns = False
    WordWrap = False
  end
  object SD: TSaveDialog
    DefaultExt = 'txt'
    FileName = '*.txt'
    Filter = '*.txt'
    InitialDir = '.'
    Left = 112
    Top = 192
  end
  object OD: TOpenDialog
    DefaultExt = 'txt'
    FileName = '*.txt'
    InitialDir = '.'
    Left = 80
    Top = 192
  end
end
