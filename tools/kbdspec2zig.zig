const std = @import("std");

pub fn main() anyerror!u8 {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);

    const raw_args = try std.process.argsAlloc(arena.allocator());
    const args = raw_args[1..];

    if (args.len < 1) {
        std.log.err("[!] Usage: kbdspec2zig <kbdspec file>", .{});
        return 1;
    }

    const input = std.fs.cwd().readFileAlloc(arena.allocator(), args[0], std.math.maxInt(usize)) catch {
        std.log.err("[!] Unable to read input file!", .{});
        return 1;
    };

    var parser = Parser.init(input);

    parser.skipWhitespace();
    try parser.expectLiteral("VERSION");
    parser.skipWhitespace();

    const version = try parser.expectInteger();
    if (version != 1) return error.InvalidVersion;
    parser.expectNewline();

    var layout_name: ?[]const u8 = null;
    var keys = std.ArrayList(Key).init(arena.allocator());
    var current_width: f32 = 1.0;
    var current_height: f32 = 1.0;
    var offset_x: f32 = 0.0;
    var offset_y: f32 = 0.0;
    var mat_row_major = true;
    var mat_row: i32 = 0;
    var mat_col: i32 = 0;
    var next_stabilizer_name: ?[]const u8 = null;

    // Now we expect a bunch of commands.
    while (parser.peek() != 0) {
        const cmd = try parser.expectWord();
        parser.skipWhitespace();

        if (equalsIgnoreCase("NAME", cmd)) {
            if (layout_name != null) return error.LayoutNameSetTwice;
            layout_name = try parser.expectString();
        } else if (equalsIgnoreCase("SET_WIDTH", cmd)) {
            current_width = try parser.expectFloat();
        } else if (equalsIgnoreCase("SET_HEIGHT", cmd)) {
            current_height = try parser.expectFloat();
        } else if (equalsIgnoreCase("SET_OFFSET", cmd)) {
            offset_x = try parser.expectFloat();
            parser.skipWhitespace();
            offset_y = try parser.expectFloat();
        } else if (equalsIgnoreCase("STABILIZER", cmd)) {
            next_stabilizer_name = try parser.expectString();
        } else if (equalsIgnoreCase("KEY", cmd)) {
            const key_name = try parser.expectString();
            parser.skipWhitespace();
            const key_width = if (isNumberChar(parser.peek())) try parser.expectFloat() else current_width;
            parser.skipWhitespace();
            const key_height = if (isNumberChar(parser.peek())) try parser.expectFloat() else current_height;

            try keys.append(.{
                .x_pos = offset_x,
                .y_pos = offset_y,
                .width = key_width,
                .height = key_height,
                .mat_row = mat_row,
                .mat_col = mat_col,
                .name = key_name,
                .id = keys.items.len,
                .stabilizer_name = next_stabilizer_name,
            });

            next_stabilizer_name = null;

            // Auto-inc width by default.
            offset_x += key_width;

            if (mat_row_major) mat_col += 1 else mat_row += 1;
        } else if (equalsIgnoreCase("MAT_ROWMAJOR", cmd)) {
            mat_row_major = true;
        } else if (equalsIgnoreCase("MAT_COLUMNMAJOR", cmd)) {
            mat_row_major = false;
        } else if (equalsIgnoreCase("MAT_INC_MAJOR", cmd)) {
            const inc_amount = if (isNumberChar(parser.peek())) try parser.expectInteger() else 1;
            if (mat_row_major) {
                mat_row += inc_amount;
                mat_col = 0;
            } else {
                mat_col += inc_amount;
                mat_row = 0;
            }
        } else if (equalsIgnoreCase("MAT_INC_MINOR", cmd)) {
            const inc_amount = if (isNumberChar(parser.peek())) try parser.expectInteger() else 1;
            if (mat_row_major) {
                mat_col += inc_amount;
            } else {
                mat_row += inc_amount;
            }
        } else return error.UnknownCommand;

        parser.expectNewline();
    }

    std.log.info("Found {d} keys", .{ keys.items.len });

    return 0;
}

const Key = struct {
    x_pos: f32,
    y_pos: f32,
    width: f32,
    height: f32,
    mat_row: i32,
    mat_col: i32,
    name: []const u8,
    id: usize,
    stabilizer_name: ?[]const u8,
};

fn equalsIgnoreCase(a: []const u8, b: []const u8) bool {
    if (a.len != b.len) return false;

    for (a) |char, i| {
        if (std.ascii.toLower(char) != std.ascii.toLower(b[i])) return false;
    }

    return true;
}

const Parser = struct {
    source: []const u8,
    start: usize = 0,
    end: usize = 0,
    line: usize = 1,

    pub fn init(source: []const u8) Parser {
        return .{
            .source = source,
        };
    }

    pub fn peek(self: *Parser) u8 {
        return if (self.end >= self.source.len) 0 else self.source[self.end];
    }

    pub fn advance(self: *Parser) void {
        self.end += 1;
    }

    fn setStartToEnd(self: *Parser) void {
        while (self.start < self.end) : (self.start += 1) {
            if (self.start < self.source.len and self.source[self.start] == '\n') {
                self.line += 1;
            }
        }
    }

    pub fn discard(self: *Parser) void {
        self.setStartToEnd();
    }

    pub fn match(self: *Parser) []const u8 {
        const str = self.source[self.start.. self.end];
        self.setStartToEnd();
        return str;
    }

    pub fn skipWhitespace(p: *Parser) void {
        while (isSpace(p.peek())) p.advance();
        p.discard();
    }

    pub fn expectNewline(p: *Parser) void {
        while (true) {
            while (isSpace(p.peek()) or p.peek() == '\n') p.advance();
            p.discard();

            // We may have hit a comment.
            if (p.peek() != '#') return;
            skipComments(p);
        }
    }

    pub fn skipComments(p: *Parser) void {
        if (p.peek() != '#') return;

        while (p.peek() != '\n' and p.peek() != 0) p.advance();
        p.discard();
    }

    pub fn expectLiteral(p: *Parser, str: []const u8) !void {
        for (str) |char| {
            if (char != p.peek()) return error.ParseKbdspec;
            p.advance();
        }

        p.discard();
    }

    pub fn expectInteger(p: *Parser) !i32 {
        if (!isNumberChar(p.peek())) return error.ParseKbdspec;
        while (isNumberChar(p.peek())) p.advance();

        return std.fmt.parseInt(i32, p.match(), 10) catch return error.ParseKbdspec;
    }

    pub fn expectFloat(p: *Parser) !f32 {
        if (!isNumberChar(p.peek())) return error.ParseKbdspec;
        while (isNumberChar(p.peek())) p.advance();
        if (p.peek() == '.') {
            p.advance();
            while (isNumberChar(p.peek())) p.advance();
        }

        return std.fmt.parseFloat(f32, p.match()) catch return error.ParseKbdspec;
    }

    pub fn expectWord(p: *Parser) ![]const u8 {
        if (!isWordChar(p.peek(), true)) return error.ParseKbdspec;
        while (isWordChar(p.peek(), false)) p.advance();
        return p.match();
    }

    pub fn expectString(p: *Parser) ![]const u8 {
        if (p.peek() != '"') return error.ParseKbdspec;
        p.advance();
        p.discard();

        while (p.peek() != '"' and p.peek() != 0) p.advance();
        const str = p.match();

        // Take care of the ending quotes.
        p.advance();
        p.discard();

        return str;
    }
};

fn isWordChar(c: u8, first_letter: bool) bool {
    return (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z') or c == '_' or (!first_letter and isNumberChar(c)); 
}

fn isNumberChar(c: u8) bool {
    return c >= '0' and c <= '9';
}

fn isSpace(c: u8) bool {
    return c == ' ' or c == '\t' or c == '\r';
}