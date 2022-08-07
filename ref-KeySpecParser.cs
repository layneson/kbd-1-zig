using System;
using System.Collections.Generic;

namespace Kbdgen
{
    public sealed class KeySpecParser
    {
        private string Source { get; }
        private int Start { get; set; }
        private int End { get; set; }
        private int Line { get; set; }

        private KeySpecParser(string source)
        {
            Source = source;
            Start = 0;
            End = 0;
            Line = 1;
        }

        private char Peek() => End >= Source.Length ? '\0' : Source[End];

        private char Advance()
        {
            var c = Peek();
            End += 1;
            return c;
        }

        private void SetStartToEnd()
        {
            for (; Start < End; Start++)
            {
                if (Start < Source.Length && Source[Start] == '\n') Line++;
            }
        }

        private void Discard() => SetStartToEnd();

        private string Match()
        {
            var str = Source.Substring(Start, End - Start);
            SetStartToEnd();
            return str;
        }

        private void ThrowParseException(string message = "") =>
            throw new ParseException(Line, Peek(), message);

        private KeyLayout DoParse(KeyProfile profile)
        {
            SkipWhitespace();

            // Expect version.
            ExpectLiteral("VERSION");
            SkipWhitespace();

            var version = ExpectInteger();
            if (version != 1) ThrowParseException($"expected version 1, found {version}");
            ExpectNewline();

            // State variables for the keyboard layout.
            string? layoutName = null;
            var keys = new List<KeyLayout.Key>();
            var currentWidth = 1.0f;
            var currentHeight = 1.0f;
            var offsetX = 0.0f;
            var offsetY = 0.0f;
            var matRowMajor = true;
            var matRow = 0;
            var matCol = 0;
            string? nextStabilizerName = null;

            // Now we expect a bunch of commands.
            while (Peek() != '\0')
            {
                var cmd = ExpectWord();
                SkipWhitespace();

                if (cmd.Equals("NAME", StringComparison.OrdinalIgnoreCase))
                {
                    if (layoutName is not null) throw new ParseException(Line, "layout name set twice");
                    layoutName = ExpectString();
                }
                else if (cmd.Equals("SET_WIDTH", StringComparison.OrdinalIgnoreCase))
                {
                    currentWidth = ExpectFloat();
                }
                else if (cmd.Equals("SET_HEIGHT", StringComparison.OrdinalIgnoreCase))
                {
                    currentHeight = ExpectFloat();
                }
                else if (cmd.Equals("SET_OFFSET", StringComparison.OrdinalIgnoreCase))
                {
                    offsetX = ExpectFloat();
                    SkipWhitespace();
                    offsetY = ExpectFloat();
                }
                else if (cmd.Equals("STABILIZER", StringComparison.OrdinalIgnoreCase))
                {
                    nextStabilizerName = ExpectString();
                }
                else if (cmd.Equals("KEY", StringComparison.OrdinalIgnoreCase))
                {
                    var keyName = ExpectString();
                    SkipWhitespace();
                    var keyWidth = IsNumberChar(Peek()) ? ExpectFloat() : currentWidth;
                    SkipWhitespace();
                    var keyHeight = IsNumberChar(Peek()) ? ExpectFloat() : currentHeight;

                    keys.Add(new KeyLayout.Key
                    {
                        XPos = offsetX,
                        YPos = offsetY,
                        Width = keyWidth,
                        Height = keyHeight,
                        MatRow = matRow,
                        MatCol = matCol,
                        Name = keyName,
                        Id = keys.Count,
                        StabilizerName = nextStabilizerName
                    });

                    nextStabilizerName = null;

                    // Auto-inc width by default.
                    offsetX += keyWidth;

                    if (matRowMajor) matCol += 1;
                    else matRow += 1;
                }
                else if (cmd.Equals("MAT_ROWMAJOR", StringComparison.OrdinalIgnoreCase))
                {
                    matRowMajor = true;
                }
                else if (cmd.Equals("MAT_COLUMNMAJOR", StringComparison.OrdinalIgnoreCase))
                {
                    matRowMajor = false;
                }
                else if (cmd.Equals("MAT_INC_MAJOR", StringComparison.OrdinalIgnoreCase))
                {
                    var incAmount = IsNumberChar(Peek()) ? ExpectInteger() : 1;
                    if (matRowMajor)
                    {
                        matRow += incAmount;
                        matCol = 0;
                    }
                    else
                    {
                        matCol += incAmount;
                        matRow = 0;
                    }
                }
                else if (cmd.Equals("MAT_INC_MINOR", StringComparison.OrdinalIgnoreCase))
                {
                    var incAmount = IsNumberChar(Peek()) ? ExpectInteger() : 1;
                    if (matRowMajor)
                    {
                        matCol += incAmount;
                    }
                    else
                    {
                        matRow += incAmount;
                    }
                }
                else ThrowParseException($"unknown command '{cmd}'");

                ExpectNewline();
            }

            return new KeyLayout(layoutName ?? "unnamed", profile, keys);
        }

        private void SkipWhitespace()
        {
            while (IsSpace(Peek())) Advance();
            Discard();
        }

        private void ExpectNewline()
        {
            while (true)
            {
                while (IsSpace(Peek()) || Peek() == '\n') Advance();
                Discard();

                // We may have hit a comment.
                if (Peek() != '#') return;
                SkipComments();
            }
        }

        private void SkipComments()
        {
            if (Peek() != '#') return;

            while (Peek() != '\n' && Peek() != '\0') Advance();
            Discard();
        }

        private void ExpectLiteral(string text)
        {
            foreach (var c in text)
            {
                if (c != Peek()) ThrowParseException($"expected '{text}'");
                Advance();
            }

            Discard();
        }

        private int ExpectInteger()
        {
            if (!IsNumberChar(Peek())) ThrowParseException("expected integer");
            while (IsNumberChar(Peek())) Advance();
            return int.Parse(Match());
        }

        private float ExpectFloat()
        {
            if (!IsNumberChar(Peek())) ThrowParseException("expected decimal number");
            while (IsNumberChar(Peek())) Advance();
            if (Peek() == '.')
            {
                Advance();
                while (IsNumberChar(Peek())) Advance();
            }

            return float.Parse(Match());
        }

        private string ExpectWord()
        {
            if (!IsWordChar(Peek())) ThrowParseException("expected word");
            while (IsWordChar(Peek())) Advance();
            return Match();
        }

        private string ExpectString()
        {
            if (Peek() != '"') ThrowParseException("expected string literal");
            Advance();
            Discard();

            while (Peek() != '"' && Peek() != '\0') Advance();
            var str = Match();

            // Take care of the ending quotes.
            Advance();
            Discard();

            return str;
        }

        private static bool IsWordChar(char c, bool firstLetter = false)
            => c >= 'a' && c <= 'z' ||
               c >= 'A' && c <= 'Z' ||
               c == '_' ||
               !firstLetter && c >= '0' && c <= '9';

        private static bool IsNumberChar(char c) => c >= '0' && c <= '9';

        private static bool IsSpace(char c) => c == ' ' || c == '\t' || c == '\r';

        public static KeyLayout Parse(string source, KeyProfile profile) => new KeySpecParser(source).DoParse(profile);

        public class ParseException : Exception
        {
            internal ParseException(int line, char c, string message)
                : base($"[{line}] invalid char '{c}': {message}")
            {
            }

            internal ParseException(int line, string message)
                : base($"[{line}] {message}")
            {
            }
        }
    }
}