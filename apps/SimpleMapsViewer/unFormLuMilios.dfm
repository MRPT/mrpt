object formLuMilios: TformLuMilios
  Left = 251
  Top = 215
  BorderStyle = bsDialog
  Caption = 'Lu & Milios consistent alignment'
  ClientHeight = 296
  ClientWidth = 566
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  Position = poScreenCenter
  PixelsPerInch = 96
  TextHeight = 13
  object Label1: TLabel
    Left = 24
    Top = 20
    Width = 127
    Height = 13
    Caption = 'Number of iterations to run:'
  end
  object btnExecute: TButton
    Left = 456
    Top = 8
    Width = 89
    Height = 33
    Caption = 'EXECUTE'
    Default = True
    TabOrder = 0
    OnClick = btnExecuteClick
  end
  object Button1: TButton
    Left = 456
    Top = 48
    Width = 89
    Height = 25
    Caption = 'CLOSE'
    ModalResult = 2
    TabOrder = 1
  end
  object edIters: TEdit
    Left = 160
    Top = 16
    Width = 65
    Height = 21
    TabOrder = 2
    Text = '5'
  end
  object logMemo: TMemo
    Left = 0
    Top = 80
    Width = 566
    Height = 216
    Align = alBottom
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Courier New'
    Font.Style = []
    ParentFont = False
    ReadOnly = True
    ScrollBars = ssBoth
    TabOrder = 3
    WordWrap = False
  end
end
